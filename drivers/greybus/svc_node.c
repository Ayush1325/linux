// SPDX-License-Identifier: GPL-2.0
/*
 * Implements a Greybus SVC node.
 *
 * Copyright (c) 2026 BeagleBoard.org Foundation
 */

#include "svc_node.h"
#include "linux/greybus/greybus_protocols.h"
#include "linux/greybus/operation.h"
#include "linux/idr.h"

#define ENDO_ID 0x4755
#define AP_INF_ID 0
#define GB_OPERATION_ID_START 1
#define GB_RESPONSE(req) (req | GB_MESSAGE_TYPE_RESPONSE)
#define GB_MSG_STRUCT(payload_type)              \
	struct payload_type##_msg {              \
		struct gb_operation_msg_hdr hdr; \
		struct payload_type payload;     \
	} __packed

GB_MSG_STRUCT(gb_svc_version_request);
GB_MSG_STRUCT(gb_svc_hello_request);
GB_MSG_STRUCT(gb_svc_dme_peer_get_response);
GB_MSG_STRUCT(gb_svc_dme_peer_set_response);
GB_MSG_STRUCT(gb_svc_pwrmon_rail_count_get_response);
GB_MSG_STRUCT(gb_svc_intf_vsys_response);
GB_MSG_STRUCT(gb_svc_intf_refclk_response);
GB_MSG_STRUCT(gb_svc_intf_unipro_response);
GB_MSG_STRUCT(gb_svc_intf_set_pwrm_request);
GB_MSG_STRUCT(gb_svc_intf_set_pwrm_response);
GB_MSG_STRUCT(gb_svc_intf_activate_response);
GB_MSG_STRUCT(gb_svc_intf_resume_response);
GB_MSG_STRUCT(gb_svc_module_inserted_request);
GB_MSG_STRUCT(gb_svc_module_removed_request);

static void gb_svc_to_ap(struct gb_svc_device *svc,
			 struct gb_operation_msg_hdr *msg)
{
	greybus_data_rcvd(svc->hd, GB_SVC_CPORT_ID, (u8 *)msg,
			  le16_to_cpu(msg->size));
}

static struct gb_operation_msg_hdr gb_request_header(struct gb_svc_device *svc,
						     u16 msg_len, u8 type,
						     bool is_oneshot)
{
	/* Operation ID 0 is reserved. */
	u16 id = (is_oneshot) ? 0 :
				ida_alloc_range(&svc->operation_id,
						GB_OPERATION_ID_START, U16_MAX,
						GFP_KERNEL);

	struct gb_operation_msg_hdr header = {
		.size = cpu_to_le16(msg_len),
		.type = type,
		.operation_id = cpu_to_le16(id),
		.result = 0,
	};

	return header;
}

static struct gb_operation_msg_hdr
gb_response_header(struct gb_host_device *hd,
		   const struct gb_operation_msg_hdr *req_hdr, u16 msg_len,
		   u8 result)
{
	struct gb_operation_msg_hdr header = {
		.size = cpu_to_le16(msg_len),
		.type = GB_RESPONSE(req_hdr->type),
		.operation_id = req_hdr->operation_id,
		.result = result,
	};

	return header;
}

static void
gb_svc_send_empty_response(struct gb_svc_device *svc,
			   const struct gb_operation_msg_hdr *req_hdr,
			   u8 result)
{
	struct gb_operation_msg_hdr msg =
		gb_response_header(svc->hd, req_hdr, sizeof(msg), result);

	gb_svc_to_ap(svc, &msg);
}

static void gb_svc_send_version(struct gb_svc_device *svc)
{
	struct gb_svc_version_request_msg msg = {
		.hdr = gb_request_header(svc, sizeof(msg), GB_SVC_TYPE_PROTOCOL_VERSION, false),
		.payload = {
			.major = GB_SVC_VERSION_MAJOR,
			.minor = GB_SVC_VERSION_MINOR,
		},
	};

	dev_info(&svc->hd->dev, "Sending SVC version request");

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_send_hello(struct gb_svc_device *svc)
{
	struct gb_svc_hello_request_msg msg = {
		.hdr = gb_request_header(svc, sizeof(msg), GB_SVC_TYPE_SVC_HELLO, false),
		.payload = {
			.endo_id = ENDO_ID,
			.interface_id = AP_INF_ID,
		},
	};

	dev_info(&svc->hd->dev, "Sending SVC Hello");

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_dme_peer_get_handler(struct gb_svc_device *svc,
					const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_dme_peer_get_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = 0,
			.attr_value = 0x0126,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_dme_peer_set_handler(struct gb_svc_device *svc,
					const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_dme_peer_set_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = 0,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void
gb_svc_pwrmon_rail_count_get_handler(struct gb_svc_device *svc,
				     const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_pwrmon_rail_count_get_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.rail_count = 0,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void
gb_svc_intf_vsys_enable_disable_handler(struct gb_svc_device *svc,
					const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_intf_vsys_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = GB_SVC_INTF_VSYS_OK,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_intf_refclk_enable_disable_handler(
	struct gb_svc_device *svc, const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_intf_refclk_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = GB_SVC_INTF_REFCLK_OK,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_intf_unipro_enable_disable_handler(
	struct gb_svc_device *svc, const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_intf_unipro_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = GB_SVC_INTF_UNIPRO_OK,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_intf_set_pwrm_handler(struct gb_svc_device *svc,
					 const struct gb_operation_msg_hdr *req)
{
	const struct gb_svc_intf_set_pwrm_request_msg *req_msg =
		(const struct gb_svc_intf_set_pwrm_request_msg *)req;
	u8 result_code =
		(req_msg->payload.tx_mode == GB_SVC_UNIPRO_HIBERNATE_MODE &&
		 req_msg->payload.rx_mode == GB_SVC_UNIPRO_HIBERNATE_MODE) ?
			GB_SVC_SETPWRM_PWR_OK :
			GB_SVC_SETPWRM_PWR_LOCAL;
	struct gb_svc_intf_set_pwrm_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.result_code = result_code,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_intf_activate_handler(struct gb_svc_device *svc,
					 const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_intf_activate_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.status = GB_SVC_OP_SUCCESS,
			.intf_type = GB_SVC_INTF_TYPE_GREYBUS,
		},
	};

	/* Maybe call a module callback */

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_intf_resume_handler(struct gb_svc_device *svc,
				       const struct gb_operation_msg_hdr *req)
{
	struct gb_svc_intf_resume_response_msg msg = {
		.hdr = gb_response_header(svc->hd, req, sizeof(msg), GB_SVC_OP_SUCCESS),
		.payload = {
			.status = GB_SVC_OP_SUCCESS,
		},
	};

	/* Maybe call a module callback */

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);
}

static void gb_svc_conn_create_handler(struct gb_svc_device *svc,
				       const struct gb_operation_msg_hdr *msg)
{
	int ret;
	u16 hd_cport, node_cport;
	struct gb_bridge_module *mod;
	const struct gb_svc_conn_create_request *req_data =
		(const struct gb_svc_conn_create_request *)((u8 *)msg +
							    sizeof(*msg));

	if (req_data->intf1_id == AP_INF_ID) {
		hd_cport = req_data->intf1_id;
		node_cport = req_data->intf2_id;
	} else if (req_data->intf2_id == AP_INF_ID) {
		hd_cport = req_data->intf2_id;
		node_cport = req_data->intf1_id;
	}

	mod = xa_load(&svc->ap_to_node_map, hd_cport);

	if (mod->create_connection) {
		ret = mod->create_connection(mod, node_cport);
		if (ret < 0) {
			return gb_svc_send_empty_response(svc, msg,
							  GB_OP_UNKNOWN_ERROR);
		}
	}

	gb_svc_send_empty_response(svc, msg, GB_OP_SUCCESS);
}

static void gb_svc_conn_destroy_handler(struct gb_svc_device *svc,
					const struct gb_operation_msg_hdr *msg)
{
	u16 hd_cport, node_cport;
	struct gb_bridge_module *mod;
	const struct gb_svc_conn_destroy_request *req_data =
		(const struct gb_svc_conn_destroy_request *)((u8 *)msg +
							     sizeof(*msg));

	if (req_data->intf1_id == AP_INF_ID) {
		hd_cport = req_data->intf1_id;
		node_cport = req_data->intf2_id;
	} else if (req_data->intf2_id == AP_INF_ID) {
		hd_cport = req_data->intf2_id;
		node_cport = req_data->intf1_id;
	}

	mod = xa_load(&svc->ap_to_node_map, hd_cport);

	if (mod->destroy_connection) {
		mod->destroy_connection(mod, node_cport);
	}

	gb_svc_send_empty_response(svc, msg, GB_OP_SUCCESS);
}

static int gb_ap_to_svc(struct gb_svc_device *svc,
			const struct gb_operation_msg_hdr *msg)
{
	u8 type = msg->type;

	switch (type) {
	case GB_SVC_TYPE_INTF_DEVICE_ID:
	case GB_SVC_TYPE_ROUTE_CREATE:
	case GB_SVC_TYPE_ROUTE_DESTROY:
	case GB_SVC_TYPE_PING:
		gb_svc_send_empty_response(svc, msg, GB_OP_SUCCESS);
		break;
	case GB_SVC_TYPE_CONN_CREATE:
		gb_svc_conn_create_handler(svc, msg);
		break;
	case GB_SVC_TYPE_CONN_DESTROY:
		gb_svc_conn_destroy_handler(svc, msg);
		break;
	case GB_SVC_TYPE_DME_PEER_GET:
		gb_svc_dme_peer_get_handler(svc, msg);
		break;
	case GB_SVC_TYPE_DME_PEER_SET:
		gb_svc_dme_peer_set_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_SET_PWRM:
		gb_svc_intf_set_pwrm_handler(svc, msg);
		break;
	case GB_SVC_TYPE_PWRMON_RAIL_COUNT_GET:
		gb_svc_pwrmon_rail_count_get_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_VSYS_ENABLE:
	case GB_SVC_TYPE_INTF_VSYS_DISABLE:
		gb_svc_intf_vsys_enable_disable_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_REFCLK_ENABLE:
	case GB_SVC_TYPE_INTF_REFCLK_DISABLE:
		gb_svc_intf_refclk_enable_disable_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_UNIPRO_ENABLE:
	case GB_SVC_TYPE_INTF_UNIPRO_DISABLE:
		gb_svc_intf_unipro_enable_disable_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_ACTIVATE:
		gb_svc_intf_activate_handler(svc, msg);
		break;
	case GB_SVC_TYPE_INTF_RESUME:
		gb_svc_intf_resume_handler(svc, msg);
		break;
	case GB_RESPONSE(GB_SVC_TYPE_PROTOCOL_VERSION):
		gb_svc_send_hello(svc);
		break;
	case GB_RESPONSE(GB_SVC_TYPE_SVC_HELLO):
		dev_info(&svc->hd->dev, "Got response to SVC Hello");
		break;
	case GB_RESPONSE(GB_SVC_TYPE_MODULE_INSERTED):
	case GB_RESPONSE(GB_SVC_TYPE_MODULE_REMOVED):
		break;
	default:
		return -ENOTSUPP;
	}

	/* Free operation id */
	if (type & GB_MESSAGE_TYPE_RESPONSE) {
		ida_free(&svc->operation_id, msg->operation_id);
	}

	return 0;
}

int gb_node_to_ap(struct gb_svc_device *svc, u8 intf_id, u16 intf_cport,
		  u8 *msg, u16 msg_len)
{
	struct gb_connection *conn =
		gb_connection_hd_find_by_intf(svc->hd, intf_id, intf_cport);
	if (!conn) {
		return -ENODEV;
	}

	greybus_data_rcvd(svc->hd, conn->hd_cport_id, msg, msg_len);
	gb_connection_put(conn);

	return 0;
}
EXPORT_SYMBOL(gb_node_to_ap);

int gb_ap_to_node(struct gb_svc_device *svc, u16 cport,
		  const struct gb_operation_msg_hdr *msg)
{
	int ret;
	struct gb_connection *conn;
	struct gb_bridge_module *intf;

	if (cport == GB_SVC_CPORT_ID) {
		return gb_ap_to_svc(svc, msg);
	}

	conn = gb_connection_hd_find(svc->hd, cport);
	if (!conn) {
		dev_err(&svc->hd->dev, "No connection at cport %d", cport);
		return -ENODEV;
	}

	intf = xa_load(&svc->ap_to_node_map, conn->intf->interface_id);
	if (!intf) {
		dev_err(&svc->hd->dev, "No module with id %d",
			conn->intf->interface_id);
		return -ENODEV;
	}

	ret = intf->write(intf, msg, conn->intf->interface_id,
			  conn->intf_cport_id);
	gb_connection_put(conn);

	return ret;
}
EXPORT_SYMBOL(gb_ap_to_node);

int gb_svc_module_insert(struct gb_svc_device *svc,
			 struct gb_bridge_module *intf)
{
	int ret;
	u32 intf_id;
	struct gb_svc_module_inserted_request_msg msg = { 
		.hdr = gb_request_header(svc, sizeof(msg), GB_SVC_TYPE_MODULE_INSERTED, false),
		.payload = {
			.intf_count = 1,
			.flags = 0,
		},
	};

	ret = xa_alloc(&svc->ap_to_node_map, &intf_id, intf,
		       XA_LIMIT(0, U8_MAX), GFP_KERNEL);
	if (ret < 0) {
		return ret;
	}

	msg.payload.primary_intf_id = intf_id;

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);

	return 0;
}
EXPORT_SYMBOL(gb_svc_module_insert);

struct gb_bridge_module *gb_svc_module_remove(struct gb_svc_device *svc,
					      uint8_t id)
{
	struct gb_svc_module_removed_request_msg msg = {
		.hdr = gb_request_header(svc, sizeof(msg), GB_SVC_TYPE_MODULE_REMOVED, false),
		.payload = {
			.primary_intf_id = id,
		},
	};

	gb_svc_to_ap(svc, (struct gb_operation_msg_hdr *)&msg);

	return xa_erase(&svc->ap_to_node_map, id);
}
EXPORT_SYMBOL(gb_svc_module_remove);

int gb_bridge_module_find(struct gb_svc_device *svc, gb_bridge_module_find_t cb)
{
	struct gb_bridge_module *intf;
	unsigned long index;

	xa_for_each(&svc->ap_to_node_map, index, intf) {
		if (cb(intf)) {
			return index;
		}
	}

	return -ENODEV;
}
EXPORT_SYMBOL(gb_bridge_module_find);

int gb_svc_device_init(struct gb_svc_device *svc, struct gb_host_device *hd)
{
	svc->hd = hd;

	ida_init(&svc->operation_id);
	xa_init_flags(&svc->ap_to_node_map, XA_FLAGS_ALLOC);

	gb_svc_send_version(svc);

	return 0;
}
EXPORT_SYMBOL(gb_svc_device_init);

void gb_svc_device_deinit(struct gb_svc_device *svc)
{
	ida_destroy(&svc->operation_id);
	xa_destroy(&svc->ap_to_node_map);
}
EXPORT_SYMBOL(gb_svc_device_deinit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ayush Singh <ayush@beagleboard.org>");
MODULE_DESCRIPTION("Greybus SVC node implementation");
