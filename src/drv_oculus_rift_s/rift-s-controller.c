/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#include <hidapi.h>
#include <string.h>

#include "rift-s-hmd.h"
#include "rift-s-protocol.h"
#include "rift-s-controller.h"

static int
update_device_types (rift_s_hmd_t *hmd, hid_device *hid) {
	int res;
	rift_s_devices_list_t dev_list;

	res = rift_s_read_devices_list (hid, &dev_list);
	if (res < 0)
		return res;


	for (int i = 0; i < dev_list.num_devices; i++) {
		rift_s_device_type_record_t *dev = dev_list.devices + i;
		int c;

		for (c = 0; c < hmd->num_active_controllers; c++) {
			if (hmd->controllers[c].device_id == dev->device_id) {
				if (hmd->controllers[c].device_type != dev->device_type) {
					hmd->controllers[c].device_type = dev->device_type;
					if (dev->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
						hmd->touch_dev[0].device_num = c;
					}
					else if (dev->device_type == RIFT_S_DEVICE_RIGHT_CONTROLLER) {
						hmd->touch_dev[1].device_num = c;
					}
				}
				break;
			}
		}
		if (c == hmd->num_active_controllers) {
			LOGW("Got a device type record for an unknown device 0x%16lx\n", dev->device_id);
		}
	}

	return 0;
}

void
rift_s_handle_controller_report (rift_s_hmd_t *hmd, hid_device *hid, const unsigned char *buf, int size)
{
	rift_s_controller_report_t report;

	if (!rift_s_parse_controller_report (&report, buf, size)) {
		rift_s_hexdump_buffer ("Invalid Controller Report", buf, size);
	}

	if (report.device_id != 0x00) {
		int i;
		rift_s_controller_state_t *ctrl = NULL;

		for (i = 0; i < hmd->num_active_controllers; i++) {
			 if (hmd->controllers[i].device_id == report.device_id) {
				 ctrl = hmd->controllers + i;
				 break;
			 }
		}

		if (ctrl == NULL) {
			if (hmd->num_active_controllers == MAX_CONTROLLERS) {
				fprintf (stderr, "Too many controllers. Can't add %08lx\n", report.device_id);
				return;
			}

			/* Add a new controller to the tracker */
			ctrl = hmd->controllers + hmd->num_active_controllers;
			hmd->num_active_controllers++;

			memset (ctrl, 0, sizeof (rift_s_controller_state_t));
			ctrl->device_id = report.device_id;
			update_device_types (hmd, hid);
			LOGI ("Found new controller 0x%16lx type %08x\n", report.device_id, ctrl->device_type);
		}

		/* If we didn't already succeed in reading the type for this device, try again */	
		if (ctrl->device_type == 0x00)
			update_device_types (hmd, hid);
	}
}
