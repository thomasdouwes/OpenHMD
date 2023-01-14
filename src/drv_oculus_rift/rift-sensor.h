// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */
#include "ohmd-video.h"
#include "ohmd-gstreamer.h"

#include "rift.h"
#include "rift-sensor-device.h"
#include "rift-sensor-pose-helper.h"
#include "rift-tracker-common.h"

#ifndef __RIFT_SENSOR_H__
#define __RIFT_SENSOR_H__

#define RIFT_SENSOR_SERIAL_LEN 32

typedef struct rift_sensor_ctx_s rift_sensor_ctx;
typedef struct rift_sensor_frame_device_state rift_sensor_frame_device_state;
typedef struct rift_sensor_analysis_frame rift_sensor_analysis_frame;

struct rift_sensor_frame_device_state {
	/* Predicted device pose and error bounds from fusion, in camera space */
	posef capture_cam_pose;
	vec3f obj_cam_pos_error;
	vec3f	obj_cam_rot_error;
	float gravity_error_rad; /* Gravity vector uncertainty in radians 0..M_PI */

	bool found_device_pose; /* Set to true when the device was found in this exposure */
	posef final_cam_pose;
	rift_pose_metrics score;
};

struct rift_sensor_analysis_frame {
	/* Index of the frame in the frames array */
	uint8_t id;

	/* Video frame data we are analysing */
	ohmd_video_frame *vframe;

	/* Exposure info from the HMD - HMD time,
	 * count and led pattern */
	bool exposure_info_valid;
	rift_tracker_exposure_info exposure_info;

	blobservation* bwobs;

	/* Device poses at capture time */
	rift_sensor_frame_device_state capture_state[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	/* Timestamp of complete frame arriving from USB */
	uint64_t frame_delivered_ts;

	/* Timestamp of fast/image analysis thread processing start */
	uint64_t image_analysis_start_ts;

	uint64_t blob_extract_finish_ts;

	/* Timestamp of fast/image analysis thread processing finish */
	uint64_t image_analysis_finish_ts;

	bool need_long_analysis;

	bool long_analysis_found_new_blobs;

	/* Timestamp of long/image analysis thread processing start */
	uint64_t long_analysis_start_ts;

	/* Timestamp of long/image analysis thread processing end */
	uint64_t long_analysis_finish_ts;
};

rift_sensor_ctx * rift_sensor_new(ohmd_context* ohmd_ctx, int id, const char *serial_no,
	rift_sensor_device *dev, rift_tracker_ctx *tracker, ohmd_gst_pipeline *debug_pipe);
void rift_sensor_free (rift_sensor_ctx *sensor_ctx);
const char *rift_sensor_serial_no (rift_sensor_ctx *sensor);
bool rift_sensor_add_device (rift_sensor_ctx *ctx, rift_tracked_device *device);
bool rift_sensor_start(rift_sensor_ctx *sensor);

void rift_sensor_set_pose(rift_sensor_ctx *sensor, posef *camera_pose);

#endif
