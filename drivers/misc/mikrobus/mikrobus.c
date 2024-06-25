// SPDX-License-Identifier: GPL-2.0:
/*
 * Copyright 2024 Ayush Singh <ayush@beagleboard.org>
 */

#define pr_fmt(fmt) "mikrobus:%s: " fmt, __func__

#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>

#define MIKROBUS_CS_CNT_MAX 12
#define MIKROBUS_CS_INVALID U32_MAX

static int of_register_mikrobus_board(struct device *dev,
				      struct device_node *np);

struct mikrobus_board {
	struct of_changeset ocs;
	int fdt_ocs_id;

	struct spi_device **spi_devs;
	size_t spi_dev_count;
};

struct mikrobus_port_spi {
	struct spi_controller *ctrl;
	u32 cs[SPI_CS_CNT_MAX];
};

/**
 * struct mikrobus_port - MikroBUS Driver
 *
 * @dev: underlying platform_device
 * @board_ocs: board device tree changeset
 * @pinctrl: mikroBUS pinctrl
 * @mikrobus_spi_cs: list of supported chipselect address and name
 * @mikrobus_spi_cs_count: length of mikrobus_spi_cs
 * @spi_ctrl: spi controller of mikroBUS connector
 * @spi_dev: spi mikroBUS board
 */
struct mikrobus_port {
	struct device dev;
	struct pinctrl *pctrl;

	struct mikrobus_port_spi *port_spi;
	struct mikrobus_board *board;
};

static const struct bus_type mikrobus_bus_type = {
	.name = "mikrobus",
};

static ssize_t new_device_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ocs, ret;
	struct device_node *np;
	struct mikrobus_port *mb = container_of(dev, struct mikrobus_port, dev);

	if (mb->board)
		return dev_err_probe(
			dev, count,
			"Device already present. Delete the original device first.");

	ret = of_overlay_fdt_apply(buf, count, &ocs, dev->of_node);
	if (ret < 0)
		return dev_err_probe(dev, count, "Failed to apply fdt");

	np = of_get_child_by_name(dev->of_node, "board");
	if (!(np && of_device_is_available(np))) {
		dev_err(dev, "Failed to register mikrobus board. Reverting...");
		goto free_node;
	}

	ret = of_register_mikrobus_board(dev->parent, np);
	of_node_put(np);
	if (ret < 0) {
		dev_err(dev, "Failed to register mikrobus board. Reverting...");
		goto revert;
	}
	mb->board->fdt_ocs_id = ocs;

	return count;

free_node:
	of_node_put(np);
revert:
	of_overlay_remove(&ocs);
	return count;
}
static DEVICE_ATTR_WO(new_device);

static struct attribute *mikrobus_port_attrs[] = { &dev_attr_new_device.attr,
						   NULL };

ATTRIBUTE_GROUPS(mikrobus_port);

static void mikrobus_port_release(struct device *dev)
{
}

static const struct device_type mikrobus_port_type = {
	.groups = mikrobus_port_groups,
	.release = mikrobus_port_release,
};

/*
 * mikrobus_pinctrl_select: Select pinctrl state for mikrobus pin
 *
 * @port: mikrobus port
 * @pinctrl_selected: pinctrl state to be selected
 */
static int mikrobus_pinctrl_select(struct device *dev,
				   const char *pinctrl_selected)
{
	int ret;
	struct pinctrl_state *state;
	struct mikrobus_port *mb = dev_get_drvdata(dev);

	state = pinctrl_lookup_state(mb->pctrl, pinctrl_selected);
	if (IS_ERR(state))
		return dev_err_probe(dev, PTR_ERR(state),
				     "failed to find state %s",
				     pinctrl_selected);

	ret = pinctrl_select_state(mb->pctrl, state);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select state %s",
				     pinctrl_selected);

	dev_dbg_ratelimited(dev, "setting pinctrl %s", pinctrl_selected);

	return 0;
}

static int of_changeset_mikrobus_spi_device(struct device *dev,
					    struct device_node *np)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);
	u32 cs[MIKROBUS_CS_CNT_MAX];
	int ret, i;

	ret = of_property_read_variable_u32_array(np, "reg", cs, 1,
						  MIKROBUS_CS_CNT_MAX);
	if (ret < 0)
		return ret;

	for (i = 0; i < ret; ++i) {
		if (mb->port_spi->cs[cs[i]] == MIKROBUS_CS_INVALID)
			return -ENOTSUPP;

		cs[i] = mb->port_spi->cs[cs[i]];
	}

	ret = of_changeset_update_prop_u32_array(&mb->board->ocs, np, "reg", cs,
						 ret);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to add reg property");

	return 0;
}

static int of_register_mikrobus_board(struct device *dev,
				      struct device_node *np)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);
	struct device_node *child, *protocol;
	struct spi_device *spi_dev;
	int ret, i;

	mb->board = devm_kmalloc(dev, sizeof(*mb->board), GFP_KERNEL);
	if (!mb->board)
		return -ENOMEM;

	of_changeset_init(&mb->board->ocs);
	mb->board->fdt_ocs_id = -1;
	mb->board->spi_dev_count = 0;
	mb->board->spi_devs = NULL;

	/* Create a changeset for the complete board */
	protocol = of_get_child_by_name(np, "spi");
	if (protocol) {
		ret = of_get_available_child_count(protocol);
		if (ret > 0) {
			mb->board->spi_dev_count = ret;

			ret = mikrobus_pinctrl_select(dev, "spi_default");
			if (ret < 0)
				goto free_np;

			for_each_available_child_of_node(protocol, child) {
				ret = of_changeset_mikrobus_spi_device(dev,
								       child);
				if (ret < 0)
					goto free_np;
			}
		}
	}
	of_node_put(protocol);
	protocol = NULL;

	ret = of_changeset_apply(&mb->board->ocs);
	if (ret < 0)
		goto free_np;

	/* Register individual devices of mikrobus board */
	protocol = of_get_child_by_name(np, "spi");
	if (protocol && mb->board->spi_dev_count) {
		mb->board->spi_devs = devm_kmalloc_array(
			dev, mb->board->spi_dev_count,
			sizeof(*mb->board->spi_devs), GFP_KERNEL);
		if (!mb->board->spi_devs) {
			ret = -ENOMEM;
			goto rollback;
		}

		i = 0;
		for_each_available_child_of_node(protocol, child) {
			spi_dev = of_register_spi_device(mb->port_spi->ctrl,
							 child);
			if (IS_ERR(spi_dev)) {
				ret = PTR_ERR(spi_dev);
				while (i > 0)
					spi_unregister_device(
						mb->board->spi_devs[--i]);
				goto rollback;
			}
			mb->board->spi_devs[i++] = spi_dev;
		}
	}
	of_node_put(protocol);

	return 0;

rollback:
	of_changeset_revert(&mb->board->ocs);
	of_changeset_destroy(&mb->board->ocs);
free_np:
	of_node_put(protocol);
	devm_kfree(dev, mb->board);
	mb->board = NULL;
	return ret;
}

static void mikrobus_board_unregister(struct mikrobus_board *dev)
{
	size_t i;

	if (dev) {
		for (i = 0; i < dev->spi_dev_count; ++i)
			spi_unregister_device(dev->spi_devs[i]);

		of_changeset_revert(&dev->ocs);
		of_changeset_destroy(&dev->ocs);

		if (dev->fdt_ocs_id >= 0)
			of_overlay_remove(&dev->fdt_ocs_id);
	}
}

static int of_register_mikrobus_port_spi(struct device *dev,
					 struct device_node *np)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);
	struct spi_controller *ctrl;
	struct device_node *temp_node;
	int ret;

	temp_node = of_parse_phandle(np, "controller", 0);
	if (!temp_node)
		return dev_err_probe(dev, -ENODEV, "Missing SPI controller");

	ret = of_property_read_variable_u32_array(np, "cs", mb->port_spi->cs, 1,
						  ARRAY_SIZE(mb->port_spi->cs));
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to read cs");
	if (ret != MIKROBUS_CS_CNT_MAX)
		return dev_err_probe(dev, -EINVAL, "Invalid SPI cs");

	ctrl = of_find_spi_controller_by_node(temp_node);
	of_node_put(temp_node);
	if (!ctrl)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to find SPI controller");

	mb->port_spi->ctrl = ctrl;

	return 0;
}

static int mikrobus_port_probe(struct platform_device *pdev)
{
	int ret;
	struct mikrobus_port *mb;
	struct device_node *np;
	struct device *dev = &pdev->dev;

	mb = devm_kmalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	dev_set_drvdata(dev, mb);

	memset(mb, 0, sizeof(*mb));
	dev_set_name(&mb->dev, dev_name(dev));
	mb->dev.id = dev->id;
	mb->dev.bus = &mikrobus_bus_type;
	mb->dev.parent = dev;
	mb->dev.of_node = dev->of_node;
	mb->dev.type = &mikrobus_port_type;
	mb->pctrl = NULL;
	mb->port_spi = NULL;
	mb->board = NULL;

	mb->pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(mb->pctrl))
		return dev_err_probe(dev, PTR_ERR(mb->pctrl),
				     "failed to get pinctrl [%ld]",
				     PTR_ERR(mb->pctrl));

	np = of_get_child_by_name(dev->of_node, "spi");
	if (np && of_device_is_available(np)) {
		mb->port_spi =
			devm_kmalloc(dev, sizeof(*mb->port_spi), GFP_KERNEL);
		if (!mb->port_spi) {
			ret = dev_err_probe(dev, -ENOMEM,
					    "Failed to allocate SPI");
			goto free_np;
		}

		ret = of_register_mikrobus_port_spi(dev, np);
		if (ret < 0) {
			dev_err(dev, "Failed to register SPI");
			goto free_spi;
		}
	}
	of_node_put(np);

	np = of_get_child_by_name(dev->of_node, "board");
	if (np && of_device_is_available(np)) {
		dev_info(dev, "Registering Mikrobus board");
		ret = of_register_mikrobus_board(dev, np);
		if (ret < 0) {
			dev_err(dev, "Failed to register mikrobus board");
			goto free_spi;
		}
	}
	of_node_put(np);

	ret = device_register(&mb->dev);
	if (ret < 0) {
		goto free_spi;
	}

	return 0;

free_spi:
	devm_kfree(dev, mb->port_spi);
	mb->port_spi = NULL;
free_np:
	of_node_put(np);
	return ret;
}

static void mikrobus_port_remove(struct platform_device *pdev)
{
	struct mikrobus_port *mb = dev_get_drvdata(&pdev->dev);

	mikrobus_board_unregister(mb->board);
	device_unregister(&mb->dev);
}

static const struct of_device_id mikrobus_port_of_match[] = {
	{ .compatible = "mikrobus-connector" },
	{},
};
MODULE_DEVICE_TABLE(of, mikrobus_port_of_match);

static struct platform_driver mikrobus_port_driver = {
	.probe = mikrobus_port_probe,
	.remove = mikrobus_port_remove,
	.driver = {
		.name = "mikrobus",
		.of_match_table = mikrobus_port_of_match,
	},
};

static int mikrobus_init(void)
{
	int ret;

	ret = bus_register(&mikrobus_bus_type);
	if (ret) {
		pr_err("bus_register failed (%d)", ret);
		return ret;
	}

	ret = platform_driver_register(&mikrobus_port_driver);
	if (ret)
		pr_err("driver register failed [%d]", ret);

	return 0;
}

module_init(mikrobus_init);

static void mikrobus_exit(void)
{
	platform_driver_unregister(&mikrobus_port_driver);
	bus_unregister(&mikrobus_bus_type);
}

module_exit(mikrobus_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ayush Singh <ayush@beagleboard.org>");
MODULE_DESCRIPTION("mikroBUS driver for linux");
MODULE_VERSION("0.1");
