/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Greybus SVC code
 *
 * Copyright 2026 BeagleBoard.org
 */

#ifndef __SVC_NODE_H
#define __SVC_NODE_H

#include <linux/greybus.h>

struct gb_svc_device {
	struct gb_host_device *hd;
	struct ida operation_id;
	struct xarray ap_to_node_map;
};

struct gb_bridge_module;

/**
 * Callback for writing to an interface
 *
 * @param controller
 * @param greybus message to send
 * @param interface id
 * @param Cport to write to
 *
 * @return 0 if successful. Negative in case of error
 */
typedef int (*gb_controller_write_callback_t)(
	struct gb_bridge_module *, const struct gb_operation_msg_hdr *, u8,
	u16);

/**
 * Callback to create new connection with a Cport in the interface
 *
 * @param controller
 * @param cport
 *
 * @return 0 if successful. Negative in case of error
 */
typedef int (*gb_controller_create_connection_t)(struct gb_bridge_module *,
						 uint16_t);

/**
 * Callback to destroy connection with a Cport in the interface
 *
 * @param controller
 * @param cport
 */
typedef void (*gb_controller_destroy_connection_t)(struct gb_bridge_module *,
						   uint16_t);

/**
 * Callback function to find a greybus interface.
 *
 * @param interface
 *
 * @returns true or false.
 */
typedef bool (*gb_bridge_module_find_t)(struct gb_bridge_module *);

/**
 * A greybus interface from bridge perspective. Can have multiple Cports
 *
 * @param write: a non-blocking write function. The ownership of message is
 * transferred.
 * @param create_connection: Called when a new connection with a cport is created. Optional.
 * @param destroy_connection: Called when an existing connection with a cport is destroyed.
 * Optional.
 * @param ctrl_data: private controller data
 */
struct gb_bridge_module {
	gb_controller_write_callback_t write;
	gb_controller_create_connection_t create_connection;
	gb_controller_destroy_connection_t destroy_connection;
	void *ctrl_data;
};

/**
 * Add greybus interface.
 *
 * @param intf
 *
 * @return 0 in case of success.
 * @return < 0 in case of error.
 */
int gb_svc_module_insert(struct gb_svc_device *svc,
			 struct gb_bridge_module *intf);

/**
 * Remove greybus interface.
 *
 * @param id: Greybus interface ID
 */
struct gb_bridge_module *gb_svc_module_remove(struct gb_svc_device *svc,
					      uint8_t id);

/**
 * Find first greybus interface in bridge by a custom function.
 *
 * @param callback
 *
 * @return grebus interface id
 * @return -1 in case no interface found.
 */
int gb_bridge_module_find(struct gb_svc_device *svc,
			  gb_bridge_module_find_t cb);

/**
 * Initialize SVC device.
 *
 * @param svc: pointer to the gb_svc_device to initialize.
 * @param hd: pointer to greybus host device.
 *
 * @return 0 in case of succes.
 * @return < 0 in case of error.
 */
int gb_svc_device_init(struct gb_svc_device *svc, struct gb_host_device *hd);

/**
 * De-initialize SVC device.
 *
 * @param svc: pointer to the gb_svc_device to de-initialize.
 */
void gb_svc_device_deinit(struct gb_svc_device *svc);

int gb_node_to_ap(struct gb_svc_device *svc, u8 intf_id, u16 intf_cport,
		  u8 *msg, u16 msg_len);

int gb_ap_to_node(struct gb_svc_device *svc, u16 cport,
		  const struct gb_operation_msg_hdr *msg);

#endif // __SVC_NODE_H
