/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef RIFT_S_CONTROLLER_H
#define RIFT_S_CONTROLLER_H

#include "rift-s.h"

#define MAX_LOG_SIZE 1024

typedef struct {
  uint64_t device_id;
  uint32_t device_type;

  /* 0x04 = new log line
   * 0x02 = parity bit, toggles each line when receiving log chars 
   * other bits, unknown */
  uint8_t log_flags;
  int log_bytes;
  uint8_t log[MAX_LOG_SIZE];

	bool imu_time_valid;
  uint32_t imu_timestamp;
  uint16_t imu_unknown_varying2;
  int16_t accel[3];
  int16_t gyro[3];

  /* 0x8, 0x0c 0x0d or 0xe block */
  uint8_t mask08;
  uint8_t buttons;
  uint8_t fingers;
  uint8_t mask0e;

  uint16_t trigger;
  uint16_t grip;

  int16_t joystick_x;
  int16_t joystick_y;

  uint8_t capsense_a_x;
  uint8_t capsense_b_y;
  uint8_t capsense_joystick;
  uint8_t capsense_trigger;

#if 0
	bool have_calibration;
	rift_touch_calibration calibration;
#endif
	fusion imu_fusion;
} rift_s_controller_state_t;

typedef struct {
	bool active_fw_read;
	int last_radio_seqnum;
	bool command_result_pending;
	int read_command_idx;
} rift_s_controller_fw_reader_state_t;

void rift_s_handle_controller_report (rift_s_hmd_t *hmd, hid_device *hid, const unsigned char *buf, int size);

#endif
