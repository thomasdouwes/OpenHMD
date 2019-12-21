// Copyright 2019, Google.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Implementation of SoftPOSIT: Simultaneous Pose and Correspondence Determination */


#ifndef SOFTPOSIT_H
#define SOFTPOSIT_H

#include <opencv2/calib3d/calib3d.hpp>

#include "debug-common.h"

typedef struct {
  std::vector<cv::Vec3d> points;
  std::vector<cv::Vec3d> normals;
  cv::Mat L_inv; // line of sight?
  cv::Vec4d pose1, pose2; // pose vectors TODO: random or initial pose?
  cv::Vec3d translation;
  cv::Mat rotation;
} Object;

Object* softposit_new_object(std::vector<cv::Vec3d> points, std::vector<cv::Vec3d> normals);
void softposit_free_object(Object* obj);


typedef struct {
  size_t width, height;
  size_t data_len;
  double assign_sum;
  double assignsq_sum;
  double slack_sum;
  double* data;
} assign_mat;


typedef struct {

  size_t num_object_points;
  double beta_init;
  double beta_final;
  double beta_update; // must be > 1, multiplied to beta until beta_final
  double small;
  double focal_length;
  double alpha;

  std::vector<Object*> objects;

  std::vector<double> correction; // correction terms (w)
  std::vector<int> occlusion_mask; // non-zero if the point is visible in the current pose. 
                                   // dot product based on normals and current pose
  assign_mat assign1;
  assign_mat assign2;

  DebugVisCallback debug_cb;
  void *debug_cb_data;
} softposit_data;

softposit_data* softposit_new();

void softposit_set_debug(softposit_data *data, DebugVisCallback debug_cb, void *cb_data);

void softposit_free(softposit_data* data);

void softposit_add_object(softposit_data* data, Object *obj);

void softposit(
  softposit_data *data,
  const std::vector<cv::Point2f> &image_points
);

#endif /* SOFTPOSIT_H */

