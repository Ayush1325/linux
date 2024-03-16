/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mikroBUS manifest definition
 * extension to Greybus Manifest Definition
 *
 * Copyright 2014-2015 Google Inc.
 * Copyright 2014-2015 Linaro Ltd.
 *
 * Released under the GPLv2 and BSD licenses.
 */

#ifndef __MIKROBUS_MANIFEST_H
#define __MIKROBUS_MANIFEST_H

#include "mikrobus_core.h"

/*
 * mikrobus_manifest_header - parse mikroBUS manifest
 *
 * @info: addon board info structure to populate with parsed information
 * @data: pointer to the manifest blob
 * @size: size of the manifest blob
 *
 * returns: number of devices on success, negative error code on failure
 */
int mikrobus_manifest_parse(struct addon_board_info *info, void *data,
			    size_t size);

#endif /* __MIKROBUS_MANIFEST_H */
