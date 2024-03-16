/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mikroBUS Driver for instantiating add-on board devices
 *
 * Copyright 2020 Vaishnav M A, BeagleBoard.org Foundation.
 * Copyright 2024 Ayush Singh <ayushdevel1325@gmail.com>
 */

#ifndef __MIKROBUS_H
#define __MIKROBUS_H

#include "linux/device.h"

#define MIKROBUS_VERSION_MAJOR 0x00
#define MIKROBUS_VERSION_MINOR 0x03

#define MIKROBUS_NUM_PINCTRL_STATE 4
#define MIKROBUS_NUM_CS 2

#define MIKROBUS_PINCTRL_PWM 0
#define MIKROBUS_PINCTRL_UART 1
#define MIKROBUS_PINCTRL_I2C 2
#define MIKROBUS_PINCTRL_SPI 3

enum mikrobus_property_type {
	MIKROBUS_PROPERTY_TYPE_MIKROBUS = 0x00,
	MIKROBUS_PROPERTY_TYPE_PROPERTY,
	MIKROBUS_PROPERTY_TYPE_GPIO,
	MIKROBUS_PROPERTY_TYPE_U8,
	MIKROBUS_PROPERTY_TYPE_U16,
	MIKROBUS_PROPERTY_TYPE_U32,
	MIKROBUS_PROPERTY_TYPE_U64,
};

enum mikrobus_pin {
	MIKROBUS_PIN_PWM = 0x00,
	MIKROBUS_PIN_INT,
	MIKROBUS_PIN_RX,
	MIKROBUS_PIN_TX,
	MIKROBUS_PIN_SCL,
	MIKROBUS_PIN_SDA,
	MIKROBUS_PIN_MOSI,
	MIKROBUS_PIN_MISO,
	MIKROBUS_PIN_SCK,
	MIKROBUS_PIN_CS,
	MIKROBUS_PIN_RST,
	MIKROBUS_PIN_AN,
	MIKROBUS_PORT_PIN_COUNT,
};

enum mikrobus_pin_state {
	MIKROBUS_STATE_INPUT = 0x01,
	MIKROBUS_STATE_OUTPUT_HIGH,
	MIKROBUS_STATE_OUTPUT_LOW,
	MIKROBUS_STATE_PWM,
	MIKROBUS_STATE_SPI,
	MIKROBUS_STATE_I2C,
	MIKROBUS_STATE_UART,
};

/*
 * board_device_info describes a single device on a mikrobus add-on
 * board, an add-on board can present one or more device to the host
 *
 * @gpio_lookup: used to provide the GPIO lookup table for
 * passing the named GPIOs to device drivers.
 * @properties: used to provide the property_entry to pass named
 * properties to device drivers, applicable only when driver uses
 * device_property_read_* calls to fetch the properties.
 * @num_gpio_resources: number of named gpio resources for the device,
 * used mainly for gpiod_lookup_table memory allocation.
 * @num_properties: number of custom properties for the device,
 * used mainly for property_entry memory allocation.
 * @protocol: used to know the type of the device and it should
 * contain one of the values defined under 'enum greybus_class_type'
 * under linux/greybus/greybus_manifest.h
 * @reg: I2C address for the device, for devices on the SPI bus
 * this field is the chip select address relative to the mikrobus
 * port:0->device chip select connected to CS pin on mikroBUS port
 *	1->device chip select connected to RST Pin on mikroBUS port
 * @mode: SPI mode
 * @max_speed_hz: SPI max speed(Hz)
 * @drv_name: device_id to match with the driver
 * @irq_type: type of IRQ trigger , match with defines in linux/interrupt.h
 * @irq: irq number relative to the mikrobus port should contain one of the
 * values defined under 'enum mikrobus_pin'
 * @id: device id starting from 1
 */
struct board_device_info {
	struct gpiod_lookup_table *gpio_lookup;
	struct property_entry *properties;
	struct list_head links;
	unsigned short num_gpio_resources;
	unsigned short num_properties;
	unsigned short protocol;
	unsigned short reg;
	unsigned int mode;
	void *dev_client;
	u32 max_speed_hz;
	char *drv_name;
	int irq_type;
	int irq;
	int id;
};

/*
 * addon_board_info describes a mikrobus add-on device the add-on
 * board, an add-on board can present one or more device to the host
 *
 * @manifest_descs: list of manifest descriptors
 * @devices: list of devices on the board
 * @pin_state: the state of each pin on the mikrobus port required
 * for the add-on board should contain one of the values defined under
 * 'enum mikrobus_pin_state' restrictions are as per mikrobus standard
 * specifications.
 * @name: add-on board name
 */
struct addon_board_info {
	struct list_head manifest_descs;
	struct list_head devices;
	u8 pin_state[MIKROBUS_PORT_PIN_COUNT];
	char *name;
};

/*
 * mikrobus_port describes the peripherals mapped to a mikrobus port.
 *
 * @chip_select: chip select number mapped to the SPI CS pin on the
 * mikrobus port and the RST pin on the mikrobus port
 * @board: pointer to the attached add-on board.
 * @spi_ctrl: SPI controller attached to the mikrobus port.
 * @i2c_adap: I2C adapter attached to the mikrobus port.
 * @gpios: GPIOs attached to the mikrobus port.
 * @pinctrl: pinctrl attached to the mikrobus port.
 * @dev: device structure for the mikrobus port.
 * @id: port id starting from 1
 */
struct mikrobus_port {
	u32 chip_select[MIKROBUS_NUM_CS];
	struct addon_board_info *board;
	struct spi_controller *spi_ctrl;
	struct i2c_adapter *i2c_adap;
	struct gpio_descs *gpios;
	struct pinctrl *pinctrl;
	struct device dev;
	int id;
};

#define to_mikrobus_port(d) container_of(d, struct mikrobus_port, dev)

#endif /* __MIKROBUS_H */
