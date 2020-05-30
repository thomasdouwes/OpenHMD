/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift S Driver - firmware JSON parsing functions */
#include "rift-s-firmware.h"
#include "../ext_deps/nxjson.h"

static bool json_read_vec3(const nx_json *nxj, const char *key, vec3f *out)
{
	const nx_json *member = nx_json_get (nxj, key);

	if (member->type != NX_JSON_ARRAY)
		return false;

	for (int i = 0; i < 3; i++) {
		const nx_json *item = nx_json_item (member, i);

		if (item->type != NX_JSON_DOUBLE)
			return false;

		out->arr[i] = item->dbl_value;
	}

	return true;
}

static bool json_read_mat4x4(const nx_json *nxj, const char *key, mat4x4f *out)
{
	const nx_json *member = nx_json_get (nxj, key);

	if (member->type != NX_JSON_ARRAY)
		return false;

	for (int i = 0; i < 12; i++) {
		const nx_json *item = nx_json_item (member, i);

		if (item->type != NX_JSON_DOUBLE)
			return false;

		out->arr[i] = item->dbl_value;
	}

	return true;
}

static bool json_read_mat3x3(const nx_json *nxj, const char *key, float out[3][3])
{
	const nx_json *member = nx_json_get (nxj, key);
	int i = 0;

	if (member->type != NX_JSON_ARRAY)
		return false;

	for (int y = 0; y < 3; y++) {
		for (int x = 0; x < 3; x++) {
			const nx_json *item = nx_json_item (member, i);

			if (item->type != NX_JSON_DOUBLE)
				return false;

			out[y][x] = item->dbl_value;
			i++;
		}
	}

	return true;
}

int rift_s_parse_imu_calibration(char *json,
		rift_s_imu_calibration *c)
{
	const nx_json* nxj, *obj, *version, *imu;
	float version_number = -1;

	nxj = nx_json_parse (json, 0);
	if (nxj == NULL)
		return -1;

	obj = nx_json_get(nxj, "FileFormat");
	if (obj->type == NX_JSON_NULL)
		goto fail;

	version = nx_json_get (obj, "Version");
	if (version->type != NX_JSON_STRING) {
		goto fail;
	}
	version_number = strtof(version->text_value, NULL);
	if (version_number != 1.0)
		goto fail;

	imu = nx_json_get(nxj, "ImuCalibration");
	if (obj->type == NX_JSON_NULL)
		goto fail;

	if (!json_read_mat4x4 (imu, "DeviceFromImu", &c->imu_to_device_transform))
		goto fail;

	obj = nx_json_get (imu, "Gyroscope");
	if (obj->type != NX_JSON_OBJECT)
		goto fail;

	if (!json_read_mat3x3(obj, "RectificationMatrix", c->gyro.rectification))
		goto fail;

	obj = nx_json_get (obj, "Offset");
	if (obj->type != NX_JSON_OBJECT)
		goto fail;

	if (!json_read_vec3(obj, "ConstantOffset", &c->gyro.offset))
		goto fail;

	obj = nx_json_get (imu, "Accelerometer");
	if (obj->type != NX_JSON_OBJECT)
		goto fail;

	if (!json_read_mat3x3(obj, "RectificationMatrix", c->accel.rectification))
		goto fail;

	obj = nx_json_get (obj, "Offset");
	if (obj->type != NX_JSON_OBJECT)
		goto fail;

	if (!json_read_vec3(obj, "OffsetAtZeroDegC", &c->accel.offset_at_0C))
		goto fail;
	if (!json_read_vec3(obj, "OffsetTemperatureCoefficient", &c->accel.temp_coeff))
		goto fail;

	nx_json_free (nxj);
	return 0;

fail:
	LOGW ("Unrecognised Rift S IMU Calibration JSON data. Version %f\n%s\n", version_number, json);
	nx_json_free (nxj);
	return -1;
}

