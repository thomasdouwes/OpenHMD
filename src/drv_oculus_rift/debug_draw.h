
#include "rift-sensor-uvc.h"

#ifndef DEBUG_DRAW_H
#define DEBUG_DRAW_H

void draw_blob_debug_stuff(rift_sensor_ctx *sensor_ctx, struct rift_sensor_uvc_stream * stream);
void draw_projected_leds(rift_sensor_ctx *sensor_ctx, rift_leds *leds, struct rift_sensor_uvc_stream * stream);

#endif