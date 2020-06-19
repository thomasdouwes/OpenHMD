/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#include <hidapi.h>
#include <string.h>
#include <assert.h>

#include "rift-s-hmd.h"
#include "rift-s-protocol.h"
#include "rift-s-controller.h"

/* Set to 1 to print controller states continuously */
#define DUMP_CONTROLLER_STATE 0

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

#if DUMP_CONTROLLER_STATE
static void
print_controller_state (rift_s_controller_state_t *ctrl)
{
	/* Dump the controller state if we see something unexpected / unknown, otherwise be quiet */
	if (ctrl->device_type == 0)
		return; // We don't know this device fully yet. Ignore it

	if (ctrl->extra_bytes_len == 0 && ctrl->mask08 == 0x50 && ctrl->mask0e == 0)
		return;

  printf ("Controller %16lx type 0x%08x IMU ts %8u v2 %x accel %6d %6d %6d gyro %6d %6d %6d | ",
      ctrl->device_id, ctrl->device_type, ctrl->imu_timestamp, ctrl->imu_unknown_varying2,
      ctrl->accel[0], ctrl->accel[1], ctrl->accel[2],
      ctrl->gyro[0], ctrl->gyro[1], ctrl->gyro[2]);

  printf ("unk %02x %02x buttons %02x fingers %02x | ",
      ctrl->mask08, ctrl->mask0e, ctrl->buttons, ctrl->fingers);
  printf ("trigger %5d grip %5d |", ctrl->trigger, ctrl->grip);
  printf ("joystick x %5d y %5d |", ctrl->joystick_x, ctrl->joystick_y);
	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		printf ("capsense x %u y %u joy %u trig %u | ",
          ctrl->capsense_a_x, ctrl->capsense_b_y,
          ctrl->capsense_joystick, ctrl->capsense_trigger);
	}
	else if (ctrl->device_type == RIFT_S_DEVICE_RIGHT_CONTROLLER) {
		printf ("capsense a %u b %u joy %u trig %u | ",
          ctrl->capsense_a_x, ctrl->capsense_b_y,
          ctrl->capsense_joystick, ctrl->capsense_trigger);
	}
	else {
		printf ("capsense ?? %u ?? %u ?? %u ?? %u | ",
          ctrl->capsense_a_x, ctrl->capsense_b_y,
          ctrl->capsense_joystick, ctrl->capsense_trigger);
	}

  if (ctrl->extra_bytes_len) {
    printf (" | extra ");
    rift_s_hexdump_buffer (NULL, ctrl->extra_bytes, ctrl->extra_bytes_len);
  }
  printf("\n");
}
#endif

static void
update_controller_state (rift_s_controller_state_t *ctrl, rift_s_controller_report_t *report)
{
#if DUMP_CONTROLLER_STATE
  bool saw_imu_update = false;
#endif

	/* Collect state updates */
	ctrl->extra_bytes_len = 0;

	for (int i = 0; i < report->num_info; i++) {
		rift_s_controller_info_block_t *info = report->info + i;

		switch (info->block_id) {
			case RIFT_S_CTRL_MASK08:
				ctrl->mask08 = info->maskbyte.val;
				break;
			case RIFT_S_CTRL_BUTTONS:
				ctrl->buttons = info->maskbyte.val;
				break;
			case RIFT_S_CTRL_FINGERS:
				ctrl->fingers = info->maskbyte.val;
				break;
			case RIFT_S_CTRL_MASK0e:
				ctrl->mask0e = info->maskbyte.val;
				break;
			case RIFT_S_CTRL_TRIGGRIP:
			{
				ctrl->trigger = (uint16_t)(info->triggrip.vals[1] & 0x0f) << 8 | info->triggrip.vals[0];
				ctrl->grip = (uint16_t)(info->triggrip.vals[1] & 0xf0) >> 4 | ((uint16_t) (info->triggrip.vals[2]) << 4);
				break;
			}
			case RIFT_S_CTRL_JOYSTICK:
				ctrl->joystick_x = info->joystick.val;
				ctrl->joystick_y = info->joystick.val >> 16;
				break;
			case RIFT_S_CTRL_CAPSENSE:
				ctrl->capsense_a_x = info->capsense.a_x;
				ctrl->capsense_b_y = info->capsense.b_y;
				ctrl->capsense_joystick = info->capsense.joystick;
				ctrl->capsense_trigger = info->capsense.trigger;
				break;
			case RIFT_S_CTRL_IMU: {
				int j;

#if DUMP_CONTROLLER_STATE
				/* print the state before updating the IMU timestamp a 2nd time */
				if (saw_imu_update)
					print_controller_state (ctrl);
				saw_imu_update = true;
#endif

				ctrl->imu_timestamp = info->imu.timestamp;
				ctrl->imu_unknown_varying2 = info->imu.unknown_varying2;
				for (j = 0; j < 3; j++) {
					ctrl->accel[j] = info->imu.accel[j];
					ctrl->gyro[j] = info->imu.gyro[j];
				}
				break;
			}
			default:
				fprintf (stderr, "Oops - invalid info block with ID %02x\n", info->block_id);
				assert ("Should not be reached!" == NULL);
				break;
		}
	}

	if (report->extra_bytes_len > 0) {
		assert (report->extra_bytes_len <= sizeof (ctrl->extra_bytes));
		memcpy (ctrl->extra_bytes, report->extra_bytes, report->extra_bytes_len);
	}
	ctrl->extra_bytes_len = report->extra_bytes_len;

#if DUMP_CONTROLLER_STATE
	print_controller_state (ctrl);
#endif

	/* Finally, update and output the log */
	if (report->flags & 0x04) {
		/* New log line is starting, reset the counter */
		ctrl->log_bytes = 0;
	}

	if (ctrl->log_flags & 0x04 || (ctrl->log_flags & 0x02) != (report->flags & 0x02)) {
		/* New log bytes in this report, collect them */
		for (int i = 0; i < 3; i++) {
			uint8_t c = report->log[i];
			if (c != '\0') {
				if (ctrl->log_bytes == (MAX_LOG_SIZE-1)) {
					/* Log line got too long... output it */
					ctrl->log[MAX_LOG_SIZE-1] = '\0';
					printf ("L	%s\n", ctrl->log);
					ctrl->log_bytes = 0;
				}
				ctrl->log[ctrl->log_bytes++] = c;
			}
			else if (ctrl->log_bytes > 0) {
				/* Found the end of the string */
				ctrl->log[ctrl->log_bytes] = '\0';
				printf ("L	%s\n", ctrl->log);
				ctrl->log_bytes = 0;
			}
		}
	}
	ctrl->log_flags = report->flags;
}

void
rift_s_handle_controller_report (rift_s_hmd_t *hmd, hid_device *hid, const unsigned char *buf, int size)
{
	rift_s_controller_report_t report;

	if (!rift_s_parse_controller_report (&report, buf, size)) {
		rift_s_hexdump_buffer ("Invalid Controller Report", buf, size);
	}

	if (report.device_id == 0x00) {
		/* Dummy report. Ignore it */
		return;
	}

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

	update_controller_state (ctrl, &report);
}
