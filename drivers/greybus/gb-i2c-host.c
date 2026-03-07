// SPDX-License-Identifier: GPL-2.0
/*
 * I2C Host driver for Greybus.
 *
 * Implements a Greybus SVC in the driver itself, with I2C being used as transport.
 *
 * Copyright (c) 2026 BeagleBoard.org Foundation
 */

#include "linux/gfp_types.h"
#include "linux/greybus/greybus_protocols.h"
#include "linux/greybus/hd.h"
#include "linux/greybus/operation.h"
#include "linux/workqueue.h"
#include "linux/workqueue_types.h"
#include <linux/greybus.h>
#include "linux/i2c.h"
#include "linux/spinlock.h"
#include "linux/spinlock_types.h"
#include <linux/greybus.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/list.h>
#include "svc_node.h"
#include <linux/delay.h>

#define GB_RESPONSE(req) (req | GB_MESSAGE_TYPE_RESPONSE)
#define GB_MSG_MAX_SIZE 256

/* The maximum number of CPorts supported by Greybus Host Device */
#define GB_MAX_CPORTS 32

struct gb_i2c_host_data {
	struct platform_device *pdev;

	struct gb_host_device *gb_host_dev;
	struct gb_svc_device gb_svc_dev;
	struct i2c_adapter *i2c_adapter;

	struct work_struct gb_msg_process_work;
	struct list_head gb_msg_list;
	spinlock_t gb_msg_lock;

	struct delayed_work gb_node_recv_work;
	struct list_head gb_node_recv_list;
	spinlock_t gb_node_recv_list_lock;
};

struct gb_node_recv_item {
	struct list_head node;
	struct gb_bridge_module *mod;
	u8 intf_id;
};

struct gb_msg_with_cport {
	__le16 cport;
	struct gb_operation_msg_hdr hdr;
} __packed;

struct gb_msg_item {
	struct list_head node;
	struct gb_msg_with_cport header;
	u8 payload[];
} __packed;

static int i2c_write_with_retry(const struct i2c_client *client, const u8 *buf,
				size_t len)
{
	size_t offset = 0;
	int ret;

	while (offset < len) {
		ret = i2c_master_send(client, buf + offset, len - offset);
		if (ret < 0) {
			return ret;
		}

		offset += ret;
	}

	return 0;
}

static int i2c_read_with_retry(const struct i2c_client *client, u8 *buf,
			       size_t len)
{
	size_t offset = 0;
	int ret;

	while (offset < len) {
		ret = i2c_master_recv(client, buf + offset, len - offset);
		if (ret < 0) {
			return ret;
		}

		offset += ret;
	}

	return 0;
}

static struct gb_msg_item *gb_message_copy(struct gb_host_device *hd, u16 cport,
					   const struct gb_message *msg)
{
	struct gb_msg_item *msg_copy;

	msg_copy = devm_kzalloc(&hd->dev, sizeof(*msg_copy) + msg->payload_size,
				GFP_KERNEL);
	if (!msg_copy) {
		return ERR_PTR(ENOMEM);
	}

	msg_copy->header.cport = cport;
	memcpy(&msg_copy->header.hdr, msg->header,
	       sizeof(msg_copy->header.hdr));
	memcpy(msg_copy->payload, msg->payload, msg->payload_size);

	return msg_copy;
}

static void gb_node_recv_msg(struct gb_i2c_host_data *data,
			     struct gb_node_recv_item *item)
{
	int ret;
	u8 *buf;
	u16 msg_len;
	struct i2c_client *client;
	struct gb_msg_with_cport msg_header;
	struct device *dev = &data->pdev->dev;

	client = item->mod->ctrl_data;

	ret = i2c_read_with_retry(client, (u8 *)&msg_header,
				  sizeof(msg_header));
	if (ret < 0) {
		return dev_err(dev, "Failed to message header. Dropping");
	}

	msg_len = le16_to_cpu(msg_header.hdr.size);
	if (msg_len == sizeof(msg_header.hdr)) {
		/* If no payload, then no need for scratch buffer */
		gb_node_to_ap(&data->gb_svc_dev, item->intf_id,
			      le16_to_cpu(msg_header.cport),
			      (u8 *)&msg_header.hdr, msg_len);
		return;
	}

	buf = devm_kzalloc(&data->pdev->dev, msg_len, GFP_KERNEL);
	if (!buf) {
		return dev_err(dev, "Failed to allocate message buffer");
	}

	memcpy(buf, &msg_header.hdr, sizeof(msg_header.hdr));
	ret = i2c_read_with_retry(client, buf + sizeof(msg_header.hdr),
				  msg_len - sizeof(msg_header.hdr));
	if (ret < 0) {
		dev_err(&data->pdev->dev, "Failed to read message payload");
		goto free_buf;
	}

	gb_node_to_ap(&data->gb_svc_dev, item->intf_id,
		      le16_to_cpu(msg_header.cport), buf, msg_len);

free_buf:
	devm_kfree(dev, buf);
}

static void gb_node_recv_cb(struct work_struct *work)
{
	struct gb_node_recv_item *item;
	struct delayed_work *dwork = to_delayed_work(work);
	struct gb_i2c_host_data *data =
		container_of(dwork, struct gb_i2c_host_data, gb_node_recv_work);

	while (1) {
		spin_lock(&data->gb_node_recv_list_lock);
		item = list_last_entry_or_null(&data->gb_node_recv_list,
					       struct gb_node_recv_item, node);
		if (!item) {
			spin_unlock(&data->gb_node_recv_list_lock);
			break;
		}
		list_del(&item->node);
		spin_unlock(&data->gb_node_recv_list_lock);

		gb_node_recv_msg(data, item);

		devm_kfree(&data->pdev->dev, item);
	}
}

static void gb_msg_process_cb(struct work_struct *work)
{
	int ret;
	struct gb_i2c_host_data *data = container_of(
		work, struct gb_i2c_host_data, gb_msg_process_work);
	struct gb_msg_item *msg;

	while (1) {
		spin_lock(&data->gb_msg_lock);
		msg = list_last_entry_or_null(&data->gb_msg_list,
					      struct gb_msg_item, node);
		if (!msg) {
			spin_unlock(&data->gb_msg_lock);
			break;
		}
		list_del(&msg->node);
		spin_unlock(&data->gb_msg_lock);

		ret = gb_ap_to_node(&data->gb_svc_dev, msg->header.cport,
				    &msg->header.hdr);
		if (ret < 0) {
			dev_err(&data->pdev->dev,
				"Failed to send greybus message: %d", ret);
		}

		devm_kfree(&data->gb_host_dev->dev, msg);
	}
}

/**
 * gb_message_send() - Send greybus message using HDLC over UART
 *
 * @hd: pointer to greybus host device
 * @cport: AP cport where message originates
 * @msg: greybus message to send
 * @mask: gfp mask
 *
 * Greybus I2C frame has the following payload:
 * 1. le16 cport
 * 2. gb_operation_msg_hdr msg_header
 * 3. u8 *msg_payload
 */
static int gb_message_send(struct gb_host_device *hd, u16 cport,
			   struct gb_message *msg, gfp_t mask)
{
	struct gb_i2c_host_data *data = dev_get_drvdata(&hd->dev);
	struct gb_msg_item *msg_copy;

	msg_copy = gb_message_copy(hd, cport, msg);

	spin_lock(&data->gb_msg_lock);
	list_add(&msg_copy->node, &data->gb_msg_list);
	spin_unlock(&data->gb_msg_lock);

	greybus_message_sent(hd, msg, GB_OP_SUCCESS);
	schedule_work(&data->gb_msg_process_work);

	return 0;
}

static void gb_message_cancel(struct gb_message *message)
{
}

static struct gb_hd_driver gb_msg_driver = {
	.message_send = gb_message_send,
	.message_cancel = gb_message_cancel,
};

static int gb_i2c_node_write_cb(struct gb_bridge_module *mod,
				const struct gb_operation_msg_hdr *msg,
				u8 intf_id, u16 cport)
{
	int ret;
	struct gb_node_recv_item *item;
	__le16 cport_id = cpu_to_le16(cport);
	struct i2c_client *client = mod->ctrl_data;
	struct gb_i2c_host_data *data = dev_get_drvdata(&client->dev);

	ret = i2c_write_with_retry(mod->ctrl_data, (const char *)&cport_id,
				   sizeof(cport_id));
	if (ret < 0) {
		return ret;
	}

	ret = i2c_write_with_retry(mod->ctrl_data, (const char *)msg,
				   le16_to_cpu(msg->size));
	if (ret < 0) {
		return ret;
	}

	if (msg->operation_id != 0) {
		item = devm_kzalloc(&data->pdev->dev, sizeof(*item),
				    GFP_KERNEL);
		if (!item) {
			return -ENOMEM;
		}

		item->mod = mod;
		item->intf_id = intf_id;

		spin_lock(&data->gb_node_recv_list_lock);
		list_add(&item->node, &data->gb_node_recv_list);
		spin_unlock(&data->gb_node_recv_list_lock);

		/* Need to add bit of delay for node to complete the message processing */
		schedule_delayed_work(&data->gb_node_recv_work,
				      msecs_to_jiffies(10));
	}

	return 0;
}

static int gb_i2c_host_new_module(struct gb_i2c_host_data *data, u16 address)
{
	int ret;
	struct i2c_client *client;
	struct gb_bridge_module *module;
	struct device *dev = &data->pdev->dev;

	module = devm_kzalloc(dev, sizeof(*module), GFP_KERNEL);
	if (!module) {
		return -ENOMEM;
	}

	client = devm_i2c_new_dummy_device(dev, data->i2c_adapter, address);
	if (IS_ERR(client)) {
		ret = -ENOMEM;
		goto free_module;
	}

	dev_set_drvdata(&client->dev, data);

	module->write = gb_i2c_node_write_cb;
	module->ctrl_data = client;

	ret = gb_svc_module_insert(&data->gb_svc_dev, module);
	if (ret < 0) {
		goto free_i2c_dev;
	}

	return ret;

free_i2c_dev:
	i2c_unregister_device(client);
free_module:
	devm_kfree(dev, module);

	return ret;
}

static ssize_t new_module_store(struct device *d, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	u16 address;
	struct gb_i2c_host_data *data = dev_get_drvdata(d);

	ret = kstrtou16(buf, 0, &address);
	if (ret < 0) {
		dev_err(d, "Failed to parse I2C address: %d", ret);
		return count;
	}

	ret = gb_i2c_host_new_module(data, address);
	if (ret < 0) {
		dev_err(d, "Failed to add greybus module: %d", ret);
	}

	return count;
}
static DEVICE_ATTR_WO(new_module);

static int gb_i2c_host_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node;
	struct gb_i2c_host_data *data;
	struct device *dev = &pdev->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	data->pdev = pdev;
	platform_set_drvdata(pdev, data);

	node = of_parse_phandle(dev->of_node, "i2c-parent", 0);
	if (!node) {
		return dev_err_probe(dev, -ENODEV, "cannot parse i2c-parent");
	}

	data->i2c_adapter = of_get_i2c_adapter_by_node(node);
	of_node_put(node);
	if (!data->i2c_adapter) {
		return dev_err_probe(dev, -ENODEV, "i2c_adapter not found");
	}

	data->gb_host_dev = gb_hd_create(&gb_msg_driver, dev, GB_MSG_MAX_SIZE,
					 GB_MAX_CPORTS);
	if (IS_ERR(data->gb_host_dev)) {
		ret = dev_err_probe(dev, PTR_ERR(data->gb_host_dev),
				    "Failed to create greybus host device");
		goto cleanup_i2c_adapter;
	}

	ret = gb_hd_add(data->gb_host_dev);
	if (ret) {
		dev_err(dev, "Failed to add greybus host device");
		goto free_gb_hd;
	}

	INIT_LIST_HEAD(&data->gb_msg_list);
	INIT_WORK(&data->gb_msg_process_work, gb_msg_process_cb);

	INIT_LIST_HEAD(&data->gb_node_recv_list);
	INIT_DELAYED_WORK(&data->gb_node_recv_work, gb_node_recv_cb);

	dev_set_drvdata(&data->gb_host_dev->dev, data);

	ret = gb_svc_device_init(&data->gb_svc_dev, data->gb_host_dev);
	if (ret) {
		dev_err(dev, "Failed to init greybus svc device");
		goto free_gb_hd;
	}

	dev_info(dev, "Probe successfull");

	return 0;

free_gb_hd:
	gb_hd_put(data->gb_host_dev);
cleanup_i2c_adapter:
	i2c_put_adapter(data->i2c_adapter);

	return ret;
}

static void gb_i2c_host_remove(struct platform_device *pdev)
{
	struct gb_i2c_host_data *data = platform_get_drvdata(pdev);

	gb_hd_put(data->gb_host_dev);
	gb_hd_del(data->gb_host_dev);

	gb_svc_device_deinit(&data->gb_svc_dev);

	i2c_put_adapter(data->i2c_adapter);
}

static const struct of_device_id gb_i2c_host_of_match[] = {
	{
		.compatible = "greybus,i2c-host",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gb_i2c_host_of_match);

static struct attribute *gb_i2c_host_attrs[] = {
	&dev_attr_new_module.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gb_i2c_host);

static struct platform_driver gb_i2c_host_driver = {
	.probe = gb_i2c_host_probe,
	.remove = gb_i2c_host_remove,
	.driver = {
		.name = "gb_i2c_host",
		.of_match_table = gb_i2c_host_of_match,
		.dev_groups = gb_i2c_host_groups,
	},
};

module_platform_driver(gb_i2c_host_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ayush Singh <ayush@beagleboard.org>");
MODULE_DESCRIPTION("I2C Host driver for Greybus");
