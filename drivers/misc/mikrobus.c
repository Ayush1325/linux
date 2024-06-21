// SPDX-License-Identifier: GPL-2.0:
/*
 * Copyright 2024 Ayush Singh <ayush@beagleboard.org>
 */

#define pr_fmt(fmt) "mikrobus:%s: " fmt, __func__

#include "linux/device.h"
#include "linux/pinctrl/consumer.h"
#include "linux/of.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>

struct mikrobus_spi_cs_item {
	const char *cs_name;
	u32 cs;
};

struct mikrobus_port {
	struct platform_device *dev;
	struct of_changeset connector;
	struct of_changeset board;
	struct pinctrl *pctrl;

	struct mikrobus_spi_cs_item *mikrobus_spi_cs;
	size_t mikrobus_spi_cs_count;
	struct spi_controller *spi_ctrl;
	struct spi_device *spi_dev;
};

static int of_register_mikrobus_board(struct mikrobus_port *mb);

static ssize_t new_device_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);
	struct device_node *np;
	char *str;
	int ret;

	str = devm_kasprintf(dev, GFP_KERNEL, "/mikrobus_boards/%*s",
			     (int)count, buf);
	if (!str) {
		dev_err(dev, "Failed to allocate path");
		goto early_exit;
	}

	np = of_find_node_by_path(str);
	if (!np) {
		dev_err(dev, "Failed to find mikroBUS board: %s", str);
		goto early_exit;
	}
	devm_kfree(dev, str);

	ret = of_changeset_add_prop_phandle_array(&mb->connector, dev->of_node,
						  "board", &np->phandle, 1);
	of_node_put(np);
	if (ret < 0) {
		dev_err(dev, "Failed to add board to changeset");
		goto early_exit;
	}

	ret = of_changeset_apply(&mb->connector);
	if (ret < 0) {
		dev_err(dev, "Failed to apply changeset");
		goto early_exit;
	}

	ret = of_register_mikrobus_board(mb);
	if (ret < 0)
		dev_err(dev, "Failed to register mikroBUS board");

early_exit:
	return count;
}
DEVICE_ATTR_WO(new_device);

static ssize_t delete_device_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);

	if (!list_empty(&mb->board.entries)) {
		of_changeset_revert(&mb->board);
		of_changeset_destroy(&mb->board);
		spi_unregister_device(mb->spi_dev);

		mb->spi_dev = NULL;
		of_changeset_init(&mb->board);
	}

	return count;
}
DEVICE_ATTR_WO(delete_device);

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

static int mikrobus_lookup_cs(const struct mikrobus_spi_cs_item cs[],
			      size_t mikrobus_spi_cs_count, const char *cs_name)
{
	for (int i = 0; i < mikrobus_spi_cs_count; ++i) {
		if (strcmp(cs_name, cs[i].cs_name) == 0)
			return cs[i].cs;
	}

	return -1;
}

static int mikrobus_spi_set_cs(struct device *dev, struct device_node *np)
{
	struct mikrobus_port *mb = dev_get_drvdata(dev);
	const char *temp_str;
	int reg_len;
	int ret, i;
	u32 *reg = NULL;

	reg_len = of_property_count_strings(np, "spi-cs");
	/* Use default cs if spi-cs property not present */
	if (reg_len <= 0) {
		dev_info(dev, "Falling back to default chipselect");

		reg_len = 1;
		reg = devm_kmalloc_array(dev, reg_len, sizeof(*reg),
					 GFP_KERNEL);
		if (!reg)
			return -ENOMEM;

		ret = mikrobus_lookup_cs(mb->mikrobus_spi_cs,
					 mb->mikrobus_spi_cs_count, "default");
		if (ret < 0)
			goto free_reg;

		reg[0] = ret;
	} else {
		reg = devm_kmalloc_array(dev, reg_len, sizeof(*reg),
					 GFP_KERNEL);
		if (!reg)
			return -ENOMEM;

		for (i = 0; i < reg_len; ++i) {
			ret = of_property_read_string_index(np, "spi-cs", i,
							    &temp_str);
			if (ret < 0)
				goto free_reg;

			ret = mikrobus_lookup_cs(mb->mikrobus_spi_cs,
						 mb->mikrobus_spi_cs_count,
						 temp_str);
			if (ret < 0)
				goto free_reg;

			reg[i] = ret;
		}
	}

	ret = of_changeset_add_prop_u32_array(&mb->board, np, "reg", reg,
					      reg_len);
	if (ret < 0)
		goto free_reg;

	ret = of_changeset_apply(&mb->board);
	if (ret < 0)
		goto free_reg;

	devm_kfree(dev, reg);
	return 0;

free_reg:
	devm_kfree(dev, reg);
	return ret;
}

static int of_register_mikrobus_device(struct mikrobus_port *mb,
				       struct device_node *np)
{
	int i, pinctrl_count, ret;
	const char *temp_str;
	struct device *dev = &mb->dev->dev;

	pinctrl_count = of_property_count_strings(np, "pinctrl-apply");
	if (pinctrl_count < 0)
		return dev_err_probe(dev, pinctrl_count,
				     "Missing required property pinctrl-apply");

	for (i = 0; i < pinctrl_count; ++i) {
		ret = of_property_read_string_index(np, "pinctrl-apply", i,
						    &temp_str);
		if (ret < 0)
			return ret;

		ret = mikrobus_pinctrl_select(dev, temp_str);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Failed to set pinctrl");
	}

	if (mb->spi_ctrl && !mb->spi_dev &&
	    of_device_is_compatible(np, "mikrobus,spi")) {
		ret = mikrobus_spi_set_cs(dev, np);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to set SPI chipselect");

		mb->spi_dev = of_register_spi_device(mb->spi_ctrl, np);
		if (IS_ERR(mb->spi_dev))
			return dev_err_probe(dev, PTR_ERR(mb->spi_dev),
					     "Failed to register SPI device");
	}

	return 0;
}

static int of_register_mikrobus_board(struct mikrobus_port *mb)
{
	struct device *dev = &mb->dev->dev;
	int board_len, i, ret;
	struct device_node *np;

	board_len = of_count_phandle_with_args(dev->of_node, "board", NULL);
	for (i = 0; i < board_len; ++i) {
		np = of_parse_phandle(dev->of_node, "board", i);
		if (!np) {
			ret = dev_err_probe(dev, -ENODEV, "Board not found");
			goto free_np;
		}

		ret = of_register_mikrobus_device(mb, np);
		if (ret < 0) {
			goto free_np;
		}

		of_node_put(np);
	}

	return 0;
free_np:
	of_node_put(np);
	return ret;
}

static int mikrobus_port_probe(struct platform_device *pdev)
{
	int ret, i;
	struct mikrobus_port *mb;
	struct device_node *np;
	struct device *dev = &pdev->dev;

	dev_info(dev, "Mikrobus Probe");

	mb = devm_kmalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	dev_set_drvdata(dev, mb);

	of_changeset_init(&mb->connector);
	of_changeset_init(&mb->board);
	mb->dev = pdev;
	mb->pctrl = NULL;
	mb->spi_ctrl = NULL;
	mb->spi_dev = NULL;
	mb->mikrobus_spi_cs = NULL;
	mb->mikrobus_spi_cs_count = 0;

	mb->pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(mb->pctrl))
		return dev_err_probe(dev, PTR_ERR(mb->pctrl),
				     "failed to get pinctrl [%ld]",
				     PTR_ERR(mb->pctrl));

	np = of_parse_phandle(dev->of_node, "spi-controller", 0);
	if (np) {
		mb->spi_ctrl = of_find_spi_controller_by_node(np);
		if (mb->spi_ctrl) {
			ret = of_property_count_u32_elems(dev->of_node,
							  "spi-cs");
			if (ret < 0) {
				dev_err(dev, "Missing property spi-cs");
				goto free_np;
			}

			mb->mikrobus_spi_cs_count = ret;

			ret = of_property_count_strings(dev->of_node,
							"spi-cs-names");
			if (ret < 0) {
				dev_err(dev, "Missing property spi-cs-names");
				goto free_np;
			}

			if (mb->mikrobus_spi_cs_count != ret) {
				ret = dev_err_probe(
					dev, -EINVAL,
					"spi-cs and spi-cs-names out of sync");
				goto free_np;
			}

			mb->mikrobus_spi_cs = devm_kmalloc_array(
				dev, mb->mikrobus_spi_cs_count,
				sizeof(*mb->mikrobus_spi_cs), GFP_KERNEL);
			if (!mb->mikrobus_spi_cs) {
				ret = -ENOMEM;
				goto free_np;
			}

			for (i = 0; i < mb->mikrobus_spi_cs_count; ++i) {
				of_property_read_u32_index(
					dev->of_node, "spi-cs", i,
					&mb->mikrobus_spi_cs->cs);
				of_property_read_string_index(
					dev->of_node, "spi-cs-names", i,
					&mb->mikrobus_spi_cs->cs_name);
			}
		}
	}
	of_node_put(np);

	ret = of_register_mikrobus_board(mb);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL,
				     "Failed to register mikrobus board");

	ret = device_create_file(dev, &dev_attr_new_device);
	if (ret < 0)
		dev_err(dev, "Failed to create sysfs entry");

	ret = device_create_file(dev, &dev_attr_delete_device);
	if (ret < 0)
		dev_err(dev, "Failed to create sysfs entry");

	return 0;

free_np:
	of_node_put(np);
	return ret;
}

static void mikrobus_port_remove(struct platform_device *pdev)
{
	struct mikrobus_port *mb = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Mikrobus Remove");

	spi_unregister_device(mb->spi_dev);
	of_changeset_revert(&mb->board);
	of_changeset_revert(&mb->connector);
	device_remove_file(&pdev->dev, &dev_attr_new_device);
	device_remove_file(&pdev->dev, &dev_attr_delete_device);
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

static const struct bus_type mikrobus_bus_type = {
	.name = "mikrobus",
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
