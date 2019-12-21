
#include "rift-sensor-tracker.h"
#include "rift-sensor-opencv.h"

#include "debug_draw.h"

#define WRITE_UINT24_BE(dest, colour) dest[0] = colour & 0xff; dest[1] = colour >> 8 & 0xff; dest[2] = colour >> 16 & 0xff

 #define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
 #define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


void
draw_rgb_marker (uint8_t *pixels, int width, int stride, int height,
    int x_pos, int y_pos, int mark_width, int mark_height, uint32_t colour)
{
  int x, y;
  int min_x = MAX (0, x_pos - mark_width/2);
  int max_x = MIN (width, x_pos + mark_width/2);
  int min_y = MAX (0, y_pos - mark_height/2);
  int max_y = MIN (height, y_pos + mark_height/2);

  if (y_pos < 0 || y_pos >= height)
    return;
  if (x_pos < 0 || x_pos >= width)
    return;

  /* Horizontal line */
  uint8_t *dest = pixels + stride * y_pos + 3 * min_x;
  for (x = 0; x < max_x - min_x; x++) {
     WRITE_UINT24_BE (dest, colour);
     dest += 3;
  }

  /* Vertical line */
  dest = pixels + stride * min_y + 3 * x_pos;
  for (y = 0; y < max_y-min_y; y++) {
     WRITE_UINT24_BE (dest, colour);
     dest += stride;
  }
}

void clamp(int *val, int max) {
  if (*val < 0)
    *val = 0;
  if (*val >= max)
    *val = max-1;
}

void clamp_rect(int *x, int *y, int *rw, int *rh, int width, int height) {
  clamp(x, width);
  clamp(y, height);
  clamp(rw, width - *x);
  clamp(rh, height - *y);
}


void
draw_rgb_rect (uint8_t *pixels, int width, int stride, int height,
    int start_x, int start_y, int box_width, int box_height, uint32_t colour)
{
  clamp_rect(&start_x, &start_y, &box_width, &box_height, width, height);

  int x, y;
  uint8_t *dest = pixels + stride * start_y + 3 * start_x;
  for (x = 0; x < box_width; x++) {
     WRITE_UINT24_BE (dest, colour);
     dest += 3;
  }

  for (y = 1; y < box_height-1; y++) {
     dest = pixels + stride * (start_y+y) + 3 * start_x;

     WRITE_UINT24_BE (dest, colour);
     dest += 3 * (box_width-1);
     WRITE_UINT24_BE (dest, colour);
  }

  dest = pixels + stride * (start_y+box_height-1) + 3 * start_x;
  for (x = 0; x < box_width; x++) {
     WRITE_UINT24_BE (dest, colour);
     dest += 3;
  }
}

void
draw_blob_debug_stuff(rift_sensor_ctx *sensor_ctx, struct rift_sensor_uvc_stream * stream) {

    if (stream->debug_frame == NULL)
      stream->debug_frame = calloc(stream->width * stream->height * 3, sizeof(unsigned char));

    int x, y;
    uint8_t *src = stream->frame; //in_frame->data[0];
    uint8_t *dest = stream->debug_frame; // out_frame->data[0];
    int width = stream->width;
    int height = stream->height;
    int in_stride = stream->stride;
    int out_stride = stream->width*3; //out_frame->info.stride[0];
    // uint8_t led_pattern_phase = sensor_ctx->led_pattern_phase;

  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      uint8_t *d = dest + x*3;
      /* Expand GRAY8 to yellow */
      d[0] = d[1] = src[x];
      d[2] = 0;
    }

    dest += out_stride;
    src += in_stride;
  }

  if (sensor_ctx->bwobs && sensor_ctx->bwobs->num_blobs > 0) {
     struct blob *sorted_blobs[MAX_BLOBS_PER_FRAME];

    //  GST_INFO_OBJECT (sensor_ctx, "Sensor %d phase %d Blobs: %d", sensor_ctx->id, led_pattern_phase, sensor_ctx->bwobs->num_blobs);
#if DUMP_BLOBS
     g_print ("Sensor %d phase %d Blobs: %d\n", sensor_ctx->id, led_pattern_phase, sensor_ctx->bwobs->num_blobs);
#endif

     /* Copy pointers into the sorted blobs array */
     for (int index = 0; index < sensor_ctx->bwobs->num_blobs; index++)
       sorted_blobs[index] = sensor_ctx->bwobs->blobs + index;

#if DUMP_BLOBS
       qsort (sorted_blobs, sensor_ctx->bwobs->num_blobs, sizeof(struct blob *), compare_blobs);
#endif

     for (int index = 0; index < sensor_ctx->bwobs->num_blobs; index++)
     {
       struct blob *b = sorted_blobs[index];
       int start_x, start_y, w, h;

#if DUMP_BLOBS
       g_print ("Sensor %d Blob[%d]: %d,%d %dx%d (age %d) id %d pattern %x (unchanged %u)\n", filter->id,
         index, b->x, b->y, b->width, b->height,
         b->age, b->led_id, b->pattern, b->pattern_age);
#endif

      start_x = b->x - b->width/2;
      start_y = b->y - b->height/2;
      w = b->width;
      h = b->height;
      clamp_rect(&start_x, &start_y, &w, &h, width, height);
      src = stream->frame + start_x + in_stride * start_y;
      dest = stream->debug_frame + 3 * start_x + out_stride * start_y;
    
      for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
          /* fill the blue channel for observed blobs */
          dest[3*x+2] = src[x];
        }
        dest += out_stride;
        src += in_stride;
      }

      /* Draw a purple box around unknown blobs. Green around recognised ones */
      draw_rgb_rect (stream->debug_frame, width, out_stride, height, start_x, start_y, b->width, b->height, b->led_id == -1 ? 0xFF00FF : 0x00FF00);
    }
#if DUMP_BLOBS
     g_print ("\n");
#endif
  }
}

void
draw_projected_leds(rift_sensor_ctx *sensor_ctx, rift_leds *leds, struct rift_sensor_uvc_stream * stream) {
  draw_projected_leds_at (sensor_ctx, leds, stream, &sensor_ctx->pose_orient, &sensor_ctx->pose_pos, 0xFF0000);
}

void
draw_projected_leds_at(rift_sensor_ctx *sensor_ctx, rift_leds *leds, struct rift_sensor_uvc_stream * stream, quatf *rot, vec3f *trans, uint32_t color1)
{
  // int x, y;
  // uint8_t *src = stream->frame; //in_frame->data[0];
  uint8_t *dest = stream->debug_frame; // out_frame->data[0];
  int width = stream->width;
  int height = stream->height;
  // int in_stride = stream->stride;
  int out_stride = stream->width*3; //out_frame->info.stride[0];
  // uint8_t led_pattern_phase = sensor_ctx->led_pattern_phase;

  int i;

  if (!sensor_ctx->bwobs) {
    return;
  }

  /* Project HMD LEDs into the image */
  rift_project_points(leds->points, leds->num_points,
      &sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs,
      rot, trans,
      sensor_ctx->led_out_points);

  /* Check how many LEDs have matching blobs in this pose,
    * if there's enough we have a good match */
  int matched_visible_blobs = 0;
  int visible_leds = 0;
  for (i = 0; i < leds->num_points; i++) {
    vec3f *p = sensor_ctx->led_out_points + i;
    int x = round(p->x);
    int y = round(p->y);

    vec3f normal, position;
    double facing_dot;

    oquatf_get_rotated(rot, &leds->points[i].pos, &position);
    ovec3f_add (trans, &position, &position);

    oquatf_get_rotated(rot, &leds->points[i].dir, &normal);
    facing_dot = ovec3f_get_dot (&position, &normal);
    if (facing_dot < -0.5) {
      /* Strongly Camera facing */
      struct blob *b = blobwatch_find_blob_at(sensor_ctx->bw, x, y);
      visible_leds++;
      if (b != NULL) {
        matched_visible_blobs++;
      }
    }
  }
  printf("  vis: %d  matched: %d  blobs: %d\r", visible_leds, matched_visible_blobs, sensor_ctx->bwobs->num_led_blobs);
  bool good_pose_match = false;
  if (visible_leds > 4 && matched_visible_blobs > 4) {
    if (sensor_ctx->bwobs->num_led_blobs < 2 * matched_visible_blobs) {
      good_pose_match = true;
      printf ("  Found good pose match - %u LEDs matched %u visible ones\r",
         matched_visible_blobs, visible_leds);
    }
  }

  for (i = 0; i < leds->num_points; i++) {
    vec3f *p = sensor_ctx->led_out_points + i;
    int x = round(p->x);
    int y = round(p->y);

    vec3f normal, position;
    double facing_dot;

    oquatf_get_rotated(rot, &leds->points[i].pos, &position);
    ovec3f_add (trans, &position, &position);

    oquatf_get_rotated(rot, &leds->points[i].dir, &normal);
    facing_dot = ovec3f_get_dot (&position, &normal);
    if (facing_dot < 0) {
      /* Camera facing */
      if (good_pose_match) {
        /* Back project LED ids into blobs if we find them and the dot product
          * shows them pointing strongly to the camera */
        struct blob *b = blobwatch_find_blob_at(sensor_ctx->bw, x, y);
        if (b != NULL && facing_dot < -0.5 && b->led_id != i) {
          /* Found a blob! */
          printf ("Marking LED %d at %d,%d\n", i, x, y);
          b->led_id = i;
        }
      }
      draw_rgb_marker (dest, width, out_stride, height, x, y, 8, 8, color1);
    } else {
      draw_rgb_marker (dest, width, out_stride, height, x, y, 8, 8, 0x202000);
    }
  }
}
