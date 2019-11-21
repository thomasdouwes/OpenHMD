// Copyright 2019, Google.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Implementation of SoftPOSIT: Simultaneous Pose and Correspondence Determination */


#ifndef SOFTPOSIT_H
#define SOFTPOSIT_H

#include <opencv2/calib3d/calib3d.hpp>

typedef struct {
  std::vector<cv::Vec3d> points;
  cv::Mat L_inv; // line of sight?
  cv::Vec4d pose1, pose2; // pose vectors TODO: random or initial pose?
  cv::Vec3d translation;
  cv::Mat rotation;
} Object;

Object* softposit_new_object(std::vector<cv::Vec3d> points);
void softposit_free_object(Object* obj);

void softposit(std::vector<Object*> objects, std::vector<cv::Point2f> image_points);

#endif /* SOFTPOSIT_H */

