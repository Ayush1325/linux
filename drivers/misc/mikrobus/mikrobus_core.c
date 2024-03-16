// SPDX-License-Identifier: GPL-2.0:
/*
 * mikroBUS driver for instantiating add-on
 * board devices with an identifier EEPROM
 *
 * Copyright 2020 Vaishnav M A, BeagleBoard.org Foundation.
 * Copyright 2024 Ayush Singh <ayushdevel1325@gmail.com>
 */

#define pr_fmt(fmt) "mikrobus:%s: " fmt, __func__

#include "linux/gpio/driver.h"
#include "linux/gpio/machine.h"
#include "linux/gpio/consumer.h"
#include "linux/greybus/greybus_manifest.h"
#include "linux/i2c.h"
#include "linux/irq.h"
#include "linux/pinctrl/consumer.h"
#include "linux/platform_device.h"
#include "linux/spi/spi.h"

#include "mikrobus_core.h"
#include "mikrobus_manifest.h"

static struct class_compat *mikrobus_port_compat_class;

static const struct bus_type mikrobus_bus_type = {
	.name = "mikrobus",
};

static int mikrobus_board_register(struct mikrobus_port *port,
				   struct addon_board_info *board);
static void mikrobus_board_unregister(struct mikrobus_port *port,
				      struct addon_board_info *board);

/*
 * mikrobus_pinctrl_select: Select pinctrl state for mikrobus pin
 *
 * @port: mikrobus port
 * @pinctrl_selected: pinctrl state to be selected
 */
static int mikrobus_pinctrl_select(struct mikrobus_port *port,
				   const char *pinctrl_selected)
{
	struct pinctrl_state *state;
	int ret;

	state = pinctrl_lookup_state(port->pinctrl, pinctrl_selected);
	if (IS_ERR(state)) {
		return dev_err_probe(&port->dev, PTR_ERR(state),
				     "failed to find state %s",
				     pinctrl_selected);
	}

	ret = pinctrl_select_state(port->pinctrl, state);
	if (ret) {
		return dev_err_probe(&port->dev, ret,
				     "failed to select state %s",
				     pinctrl_selected);
	}
	dev_dbg(&port->dev, "setting pinctrl %s", pinctrl_selected);

	return 0;
}

/*
 * mikrobus_pinctrl_setup: Setup mikrobus pins to either default of gpio
 *
 * @port: mikrobus port
 * @board: mikrobus board or NULL for default state
 *
 * returns 0 on success, negative error code on failure
 */
static int mikrobus_pinctrl_setup(struct mikrobus_port *port,
				  struct addon_board_info *board)
{
	int ret;

	if (!board || board->pin_state[MIKROBUS_PIN_PWM] == MIKROBUS_STATE_PWM)
		ret = mikrobus_pinctrl_select(port, "pwm_default");
	else
		ret = mikrobus_pinctrl_select(port, "pwm_gpio");
	if (ret)
		return ret;

	if (!board || board->pin_state[MIKROBUS_PIN_RX] == MIKROBUS_STATE_UART)
		ret = mikrobus_pinctrl_select(port, "uart_default");
	else
		ret = mikrobus_pinctrl_select(port, "uart_gpio");
	if (ret)
		return ret;

	if (!board || board->pin_state[MIKROBUS_PIN_SCL] == MIKROBUS_STATE_I2C)
		ret = mikrobus_pinctrl_select(port, "i2c_default");
	else
		ret = mikrobus_pinctrl_select(port, "i2c_gpio");
	if (ret)
		return ret;

	if (!board || board->pin_state[MIKROBUS_PIN_MOSI] == MIKROBUS_STATE_SPI)
		ret = mikrobus_pinctrl_select(port, "spi_default");
	else
		ret = mikrobus_pinctrl_select(port, "spi_gpio");

	return ret;
}

/*
 * new_device_store: Expose sysfs entry for adding new board
 *
 * new_device_store: Allows userspace to add mikroBUS boards that lack 1-wire
 * EEPROM for board identification by manually passing mikroBUS manifest
 */
static ssize_t new_device_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);
	struct addon_board_info *board;
	int ret;

	if (port->board)
		return dev_err_probe(dev, -EBUSY,
				     "already has board registered");

	board = devm_kzalloc(&port->dev, sizeof(*board), GFP_KERNEL);
	if (!board)
		return -ENOMEM;

	INIT_LIST_HEAD(&board->manifest_descs);
	INIT_LIST_HEAD(&board->devices);

	ret = mikrobus_manifest_parse(board, (void *)buf, count);
	if (ret < 0) {
		ret = dev_err_probe(dev, -EINVAL, "failed to parse manifest");
		goto err_free_board;
	}

	ret = mikrobus_board_register(port, board);
	if (ret) {
		ret = dev_err_probe(dev, -EINVAL, "failed to register board %s",
				    board->name);
		goto err_free_board;
	}

	return count;

err_free_board:
	devm_kfree(&port->dev, board);
	return ret;
}
static DEVICE_ATTR_WO(new_device);

/*
 * delete_device_store: Expose sysfs entry for deleting board
 */
static ssize_t delete_device_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mikrobus_port *port = to_mikrobus_port(dev);

	if (!port->board)
		return dev_err_probe(dev, -ENODEV,
				     "does not have registered boards");

	mikrobus_board_unregister(port, port->board);
	return count;
}
static DEVICE_ATTR_IGNORE_LOCKDEP(delete_device, 0200, NULL,
				  delete_device_store);

static struct attribute *mikrobus_port_attrs[] = { &dev_attr_new_device.attr,
						   &dev_attr_delete_device.attr,
						   NULL };
ATTRIBUTE_GROUPS(mikrobus_port);

static void mikrobus_port_release(struct device *dev)
{
}

static const struct device_type mikrobus_port_type = {
	.groups = mikrobus_port_groups,
	.release = mikrobus_port_release,
};

static int mikrobus_irq_get(struct mikrobus_port *port, int irqno, int irq_type)
{
	int irq;

	if (irqno > port->gpios->ndescs - 1)
		return dev_err_probe(&port->dev, -ENODEV,
				     "GPIO %d does not exist", irqno);

	irq = gpiod_to_irq(port->gpios->desc[irqno]);
	if (irq < 0)
		return dev_err_probe(&port->dev, -EINVAL,
				     "could not get irq %d", irqno);

	irq_set_irq_type(irq, irq_type);

	return irq;
}

static int mikrobus_gpio_setup(struct gpio_desc *gpio, int gpio_state)
{
	switch (gpio_state) {
	case MIKROBUS_STATE_INPUT:
		return gpiod_direction_input(gpio);
	case MIKROBUS_STATE_OUTPUT_HIGH:
		return gpiod_direction_output(gpio, 1);
	case MIKROBUS_STATE_OUTPUT_LOW:
		return gpiod_direction_output(gpio, 0);
	case MIKROBUS_STATE_PWM:
	case MIKROBUS_STATE_SPI:
	case MIKROBUS_STATE_I2C:
	default:
		return 0;
	}
}

static char *mikrobus_gpio_chip_name_get(struct mikrobus_port *port, int gpio)
{
	struct gpio_chip *gpiochip;

	if (gpio > port->gpios->ndescs - 1)
		return NULL;

	gpiochip = gpiod_to_chip(port->gpios->desc[gpio]);
	return kmemdup(gpiochip->label, strlen(gpiochip->label), GFP_KERNEL);
}

static int mikrobus_gpio_hwnum_get(struct mikrobus_port *port, int gpio)
{
	struct gpio_chip *gpiochip;

	if (gpio > port->gpios->ndescs - 1)
		return -ENODEV;

	gpiochip = gpiod_to_chip(port->gpios->desc[gpio]);
	return desc_to_gpio(port->gpios->desc[gpio]) - gpiochip->base;
}

static void mikrobus_board_device_release_all(struct addon_board_info *info)
{
	struct board_device_info *dev, *next;

	list_for_each_entry_safe(dev, next, &info->devices, links) {
		list_del(&dev->links);
		kfree(dev);
	}
}

static int mikrobus_device_register(struct mikrobus_port *port,
				    struct board_device_info *dev,
				    char *board_name)
{
	struct gpiod_lookup_table *lookup;
	struct spi_board_info *spi_info;
	struct i2c_board_info *i2c_info;
	struct platform_device *pdev;
	struct fwnode_handle *fwnode;
	struct spi_device *spi;
	struct i2c_client *i2c;
	int i, ret;

	dev_info(&port->dev, "registering device : %s", dev->drv_name);

	if (dev->gpio_lookup) {
		lookup = dev->gpio_lookup;

		switch (dev->protocol) {
		case GREYBUS_PROTOCOL_SPI:
			lookup->dev_id = kasprintf(GFP_KERNEL, "%s.%u",
						   dev->drv_name,
						   port->chip_select[dev->reg]);
			break;
		case GREYBUS_PROTOCOL_RAW:
			lookup->dev_id = kasprintf(GFP_KERNEL, "%s.%u",
						   dev->drv_name, dev->reg);
			break;
		default:
			lookup->dev_id = kmemdup(dev->drv_name,
						 strlen(dev->drv_name),
						 GFP_KERNEL);
		}

		dev_info(&port->dev, "adding lookup table : %s",
			 lookup->dev_id);

		for (i = 0; i < dev->num_gpio_resources; i++) {
			lookup->table[i].key = mikrobus_gpio_chip_name_get(
				port, lookup->table[i].chip_hwnum);
			lookup->table[i].chip_hwnum = mikrobus_gpio_hwnum_get(
				port, lookup->table[i].chip_hwnum);
		}

		gpiod_add_lookup_table(lookup);
	}

	switch (dev->protocol) {
	case GREYBUS_PROTOCOL_SPI:
		spi_info =
			devm_kzalloc(&port->dev, sizeof(*spi_info), GFP_KERNEL);
		strscpy_pad(spi_info->modalias, dev->drv_name,
			    sizeof(spi_info->modalias));
		if (dev->irq)
			spi_info->irq =
				mikrobus_irq_get(port, dev->irq, dev->irq_type);
		if (dev->properties) {
			fwnode = fwnode_create_software_node(dev->properties,
							     NULL);
			spi_info->swnode = to_software_node(fwnode);
		}
		spi_info->chip_select = port->chip_select[dev->reg];
		spi_info->max_speed_hz = dev->max_speed_hz;
		spi_info->mode = dev->mode;

		spi = spi_new_device(port->spi_ctrl, spi_info);
		devm_kfree(&port->dev, spi_info);
		if (!spi)
			return dev_err_probe(&port->dev, -ENODEV,
					     "failed to register spi device");
		dev->dev_client = (void *)spi;
		break;
	case GREYBUS_PROTOCOL_I2C:
		i2c_info =
			devm_kzalloc(&port->dev, sizeof(*i2c_info), GFP_KERNEL);
		if (!i2c_info)
			return -ENOMEM;

		strscpy_pad(i2c_info->type, dev->drv_name,
			    sizeof(i2c_info->type));
		if (dev->irq)
			i2c_info->irq =
				mikrobus_irq_get(port, dev->irq, dev->irq_type);
		if (dev->properties) {
			fwnode = fwnode_create_software_node(dev->properties,
							     NULL);
			i2c_info->swnode = to_software_node(fwnode);
		}
		i2c_info->addr = dev->reg;

		i2c = i2c_new_client_device(port->i2c_adap, i2c_info);
		devm_kfree(&port->dev, i2c_info);
		if (IS_ERR(dev->dev_client))
			return dev_err_probe(&port->dev,
					     PTR_ERR(dev->dev_client),
					     "failed to register i2c device");
		dev->dev_client = (void *)i2c;
		break;
	case GREYBUS_PROTOCOL_RAW:
		pdev = platform_device_alloc(dev->drv_name, 0);
		if (!pdev)
			return -ENOMEM;

		if (dev->properties) {
			ret = device_create_managed_software_node(
				&pdev->dev, dev->properties, NULL);
			if (ret)
				return dev_err_probe(
					&port->dev, ret,
					"failed to create software node");
		}
		ret = platform_device_add(dev->dev_client);
		if (ret)
			return dev_err_probe(
				&port->dev, ret,
				"failed to register platform device");
		dev->dev_client = (void *)pdev;
		break;
	case GREYBUS_PROTOCOL_UART:
	default:
		return -EINVAL;
	}

	return 0;
}

static void mikrobus_device_unregister(struct mikrobus_port *port,
				       struct board_device_info *dev,
				       char *board_name)
{
	dev_info(&port->dev, "removing device %s", dev->drv_name);
	if (dev->gpio_lookup) {
		gpiod_remove_lookup_table(dev->gpio_lookup);
		kfree(dev->gpio_lookup->dev_id);
		kfree(dev->gpio_lookup);
	}

	kfree(dev->properties);

	switch (dev->protocol) {
	case GREYBUS_PROTOCOL_SPI:
		spi_unregister_device((struct spi_device *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_I2C:
		i2c_unregister_device((struct i2c_client *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_RAW:
		platform_device_unregister(
			(struct platform_device *)dev->dev_client);
		break;
	case GREYBUS_PROTOCOL_UART:
		break;
	}
}

static int mikrobus_board_register(struct mikrobus_port *port,
				   struct addon_board_info *board)
{
	struct board_device_info *devinfo, *next;
	int ret, i;

	if (WARN_ON(list_empty(&board->devices)))
		return false;

	if (port->pinctrl) {
		ret = mikrobus_pinctrl_setup(port, board);
		if (ret)
			dev_err(&port->dev,
				"failed to setup pinctrl state [%d]", ret);
	}

	if (port->gpios) {
		for (i = 0; i < port->gpios->ndescs; i++) {
			ret = mikrobus_gpio_setup(port->gpios->desc[i],
						  board->pin_state[i]);
			if (ret)
				dev_err(&port->dev,
					"failed to setup gpio %d, state %d", i,
					board->pin_state[i]);

			gpiochip_free_own_desc(port->gpios->desc[i]);
		}
	}

	list_for_each_entry_safe(devinfo, next, &board->devices, links)
		mikrobus_device_register(port, devinfo, board->name);

	port->board = board;
	return 0;
}

static void mikrobus_board_unregister(struct mikrobus_port *port,
				      struct addon_board_info *board)
{
	struct board_device_info *devinfo, *next;

	if (WARN_ON(list_empty(&board->devices)))
		return;

	list_for_each_entry_safe(devinfo, next, &board->devices, links)
		mikrobus_device_unregister(port, devinfo, board->name);

	mikrobus_board_device_release_all(board);
	devm_kfree(&port->dev, board);
	port->board = NULL;
}

static int mikrobus_port_register(struct mikrobus_port *port)
{
	int ret;

	port->dev.bus = &mikrobus_bus_type;
	port->dev.type = &mikrobus_port_type;
	dev_set_name(&port->dev, "mikrobus-%d", port->id);

	dev_info(&port->dev, "registering port %s", dev_name(&port->dev));

	ret = device_register(&port->dev);
	if (ret)
		return dev_err_probe(&port->dev, ret,
				     "port '%d': can't register device (%d)",
				     port->id, ret);

	ret = class_compat_create_link(mikrobus_port_compat_class, &port->dev,
				       port->dev.parent);
	if (ret)
		dev_warn(&port->dev,
			 "failed to create compatibility class link");

	return ret;
}

static void mikrobus_port_delete(struct mikrobus_port *port)
{
	if (port->board)
		return dev_err(
			&port->dev,
			"attempting to delete port with registered boards, port [%s]",
			dev_name(&port->dev));

	class_compat_remove_link(mikrobus_port_compat_class, &port->dev,
				 port->dev.parent);

	devm_pinctrl_put(port->pinctrl);
	put_device(&port->spi_ctrl->dev);
	gpiod_put_array(port->gpios);
	put_device(&port->i2c_adap->dev);

	device_unregister(&port->dev);
	memset(&port->dev, 0, sizeof(port->dev));
}

static int mikrobus_port_probe_pinctrl_setup(struct mikrobus_port *port)
{
	struct device *dev = port->dev.parent;
	struct pinctrl_state *state;
	int ret;

	state = pinctrl_lookup_state(port->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(state))
		return dev_err_probe(dev, PTR_ERR(state),
				     "failed to find state %s",
				     PINCTRL_STATE_DEFAULT);

	ret = pinctrl_select_state(port->pinctrl, state);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to select state %s",
				     PINCTRL_STATE_DEFAULT);

	ret = mikrobus_pinctrl_setup(port, NULL);
	if (ret)
		dev_err(dev, "failed to select pinctrl states [%d]", ret);

	return ret;
}

static int mikrobus_port_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mikrobus_port *port;
	struct device_node *np;
	int ret;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->dev.parent = dev;
	port->dev.of_node = pdev->dev.of_node;

	/* port id */
	port->id = of_alias_get_id(dev->of_node, "mikrobus");
	if (port->id) {
		ret = dev_err_probe(dev, -EINVAL, "invalid mikrobus id");
		goto err_port;
	}

	/* I2C setup */
	np = of_parse_phandle(dev->of_node, "i2c-adapter", 0);
	if (!np) {
		ret = dev_err_probe(dev, -ENODEV, "cannot parse i2c-adapter");
		goto err_port;
	}
	port->i2c_adap = of_find_i2c_adapter_by_node(np);
	of_node_put(np);
	if (!port->i2c_adap) {
		ret = dev_err_probe(dev, -ENODEV, "cannot find i2c adapter");
		goto err_port;
	}

	/* GPIO setup */
	port->gpios = gpiod_get_array(dev, "mikrobus", GPIOD_OUT_LOW);
	if (IS_ERR(port->gpios)) {
		ret = dev_err_probe(dev, PTR_ERR(port->gpios),
				    "failed to get gpio array [%ld]",
				    PTR_ERR(port->gpios));
		goto free_i2c;
	}

	/* SPI setup */
	np = of_parse_phandle(dev->of_node, "spi-controller", 0);
	if (!np) {
		ret = dev_err_probe(dev, -ENODEV,
				    "cannot parse spi-controller");
		goto free_gpio;
	}
	port->spi_ctrl = of_find_spi_controller_by_node(np);
	of_node_put(np);
	if (!port->spi_ctrl) {
		ret = dev_err_probe(dev, -ENODEV, "cannot find spi controller");
		goto free_gpio;
	}
	ret = device_property_read_u32_array(dev, "spi-cs", port->chip_select,
					     MIKROBUS_NUM_CS);
	if (ret) {
		dev_err(dev, "failed to get spi-cs [%d]", ret);
		goto free_spi;
	}

	/* pinctrl setup */
	port->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(port->pinctrl)) {
		ret = dev_err_probe(dev, PTR_ERR(port->pinctrl),
				    "failed to get pinctrl [%ld]",
				    PTR_ERR(port->pinctrl));
		goto free_spi;
	}
	ret = mikrobus_port_probe_pinctrl_setup(port);
	if (ret) {
		dev_err(dev, "failed to setup pinctrl [%d]", ret);
		goto free_pinctrl;
	}

	/* TODO: UART */
	/* TODO: PWM */

	ret = mikrobus_port_register(port);
	if (ret) {
		dev_err(dev, "port : can't register port [%d]", ret);
		goto free_pinctrl;
	}

	platform_set_drvdata(pdev, port);

	return 0;

free_pinctrl:
	devm_pinctrl_put(port->pinctrl);
free_spi:
	put_device(&port->spi_ctrl->dev);
free_gpio:
	gpiod_put_array(port->gpios);
free_i2c:
	put_device(&port->i2c_adap->dev);
err_port:
	put_device(&port->dev);
	return ret;
}

static int mikrobus_port_remove(struct platform_device *pdev)
{
	struct mikrobus_port *port = platform_get_drvdata(pdev);

	mikrobus_port_delete(port);
	return 0;
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

	mikrobus_port_compat_class = class_compat_register("mikrobus-port");
	if (!mikrobus_port_compat_class) {
		ret = -ENOMEM;
		pr_err("class_compat register failed (%d)", ret);
		goto class_err;
	}

	ret = platform_driver_register(&mikrobus_port_driver);
	if (ret)
		pr_err("driver register failed [%d]", ret);

	return ret;

class_err:
	bus_unregister(&mikrobus_bus_type);
	return ret;
}
subsys_initcall(mikrobus_init);

static void mikrobus_exit(void)
{
	platform_driver_unregister(&mikrobus_port_driver);
	bus_unregister(&mikrobus_bus_type);
	class_compat_unregister(mikrobus_port_compat_class);
}
module_exit(mikrobus_exit);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_AUTHOR("Ayush Singh <ayushdevel1325@beagleboard.org>");
MODULE_DESCRIPTION("mikroBUS main module");
MODULE_LICENSE("GPL");
