/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __RTFT_S_FIRMWARE__
#define __RTFT_S_FIRMWARE__

#include "../openhmdi.h"

typedef enum {
		RIFT_S_FIRMWARE_BLOCK_SERIAL_NUM = 0x0B,
		RIFT_S_FIRMWARE_BLOCK_THRESHOLD = 0xD,
		RIFT_S_FIRMWARE_BLOCK_IMU_CALIB = 0xE,
		RIFT_S_FIRMWARE_BLOCK_CAMERA_CALIB = 0xF,
		RIFT_S_FIRMWARE_BLOCK_DISPLAY_COLOR_CALIB = 0x10,
		RIFT_S_FIRMWARE_BLOCK_LENS_CALIB = 0x12
} rift_s_firmware_block;

typedef struct {
	mat4x4f imu_to_device_transform;

	struct {
		float rectification[3][3];
		vec3f offset;
	} gyro;

	struct {
		float rectification[3][3];
		vec3f offset_at_0C;
		vec3f temp_coeff;
	} accel;
} rift_s_imu_calibration;

int rift_s_parse_imu_calibration(char *json, rift_s_imu_calibration *c);
#endif
