#include "rift.h"
#include <libusb.h>
#include "rift-sensor-uvc.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"
#include "ohmd-pipewire.h"
#include "kalman.h"

#define MAX_SENSORS 4
#define MAX_DEVICES 3

#define MAX_OBJECT_LEDS 64


typedef struct rift_tracker_ctx_s rift_tracker_ctx;
typedef struct rift_sensor_ctx_s rift_sensor_ctx;


struct rift_sensor_ctx_s
{
  int id;
  char serial_no[32];
  rift_tracker_ctx *tracker;

  libusb_device_handle *usb_devh;
  int stream_started;
  struct rift_sensor_uvc_stream stream;
  uint64_t frame_sof_ts;
  uint64_t led_pattern_sof_ts;
  uint8_t led_pattern_phase;
  struct blobwatch* bw;
	struct blobservation* bwobs;

  dmat3 camera_matrix;
  double dist_coeffs[4];

  kalman_pose *pose_filter;
  vec3f pose_pos;
  quatf pose_orient;

	vec3f led_out_points[MAX_OBJECT_LEDS];

  ohmd_pw_video_stream *debug_vid;
  ohmd_pw_debug_stream *debug_metadata;
};

typedef struct rift_tracked_device_s rift_tracked_device;

struct rift_tracked_device_s
{
	fusion *fusion;
};

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
  libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	ohmd_thread* usb_thread;
	int usb_completed;

  rift_leds *leds;
  uint8_t led_pattern_phase;
  uint64_t led_pattern_phase_ts;

  rift_sensor_ctx *sensors[MAX_SENSORS];
  uint8_t n_sensors;

  rift_tracked_device devices[MAX_DEVICES];
};


rift_tracker_ctx *rift_sensor_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5], rift_leds *leds);

void rift_sensor_tracker_add_device (rift_tracker_ctx *ctx, int device_id, fusion *f);
void rift_sensor_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase);
void rift_sensor_tracker_free (rift_tracker_ctx *ctx);
