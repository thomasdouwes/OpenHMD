/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include "softposit.h"

using namespace std;

extern "C" {
#include <stdio.h>

#include "rift-sensor-opencv.h"
#include "rift-sensor-blobwatch.h"
}

static void
quatf_to_3x3 (cv::Mat &mat, quatf *me) {
        mat.at<double>(0,0) = 1 - 2 * me->y * me->y - 2 * me->z * me->z;
        mat.at<double>(0,1) =     2 * me->x * me->y - 2 * me->w * me->z;
        mat.at<double>(0,2) =     2 * me->x * me->z + 2 * me->w * me->y;

        mat.at<double>(1,0) =     2 * me->x * me->y + 2 * me->w * me->z;
        mat.at<double>(1,1) = 1 - 2 * me->x * me->x - 2 * me->z * me->z;
        mat.at<double>(1,2) =     2 * me->y * me->z - 2 * me->w * me->x;

        mat.at<double>(2,0) =     2 * me->x * me->z - 2 * me->w * me->y;
        mat.at<double>(2,1) =     2 * me->y * me->z + 2 * me->w * me->x;
        mat.at<double>(2,2) = 1 - 2 * me->x * me->x - 2 * me->y * me->y;
}

extern "C" bool estimate_initial_pose(struct blob *blobs, int num_blobs,
    rift_led *leds, int num_led_pos,
    dmat3 *camera_matrix, double dist_coeffs[4],
    quatf *rot, vec3f *trans, int *num_leds_out, bool use_extrinsic_guess)
{
	int i, j;
	int num_leds = 0;
	uint64_t taken = 0;
	int flags = CV_ITERATIVE;
	cv::Mat inliers;
	int iterationsCount = 100;
	float reprojectionError = 0.1;
	float confidence = 0.999;
	cv::Mat fishK = cv::Mat(3, 3, CV_64FC1, camera_matrix->m);
	cv::Mat fishDistCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);
	cv::Mat dummyK = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat dummyD = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	// cv::Rodrigues(R, rvec);

	//cout << "R = " << R << ", rvec = " << rvec << endl;

	/* count identified leds */
	for (i = 0; i < num_blobs; i++) {
		if (blobs[i].led_id < 0)
			continue;
		if (taken & (1ULL << blobs[i].led_id))
			continue;
		taken |= (1ULL << blobs[i].led_id);
		num_leds++;
	}
	*num_leds_out = num_leds;

	if (num_leds < 4)
		return false;

	std::vector<cv::Vec3d> list_points3d(num_leds);
	std::vector<cv::Vec3d> list_points3d_all(5 /*num_led_pos*/);
	std::vector<cv::Point2f> list_points2d(num_leds);
	std::vector<cv::Point2f> list_points2d_undistorted(num_leds);

	taken = 0;
	for (i = 0, j = 0; i < num_blobs && j < num_leds; i++) {
		if (blobs[i].led_id < 0)
			continue;
		if (taken & (1ULL << blobs[i].led_id))
			continue;
		taken |= (1ULL << blobs[i].led_id);
		list_points3d[j][0] = leds[blobs[i].led_id].pos.x;
		list_points3d[j][1] = leds[blobs[i].led_id].pos.y;
		list_points3d[j][2] = leds[blobs[i].led_id].pos.z;
		list_points2d[j].x = blobs[i].x;
		list_points2d[j].y = blobs[i].y;
		j++;

		LOGD ("LED %d at %d,%d (3D %f %f %f)\n",
		    blobs[i].led_id, blobs[i].x, blobs[i].y,
		    leds[blobs[i].led_id].pos.x,
		    leds[blobs[i].led_id].pos.y,
		    leds[blobs[i].led_id].pos.z);
	}

	num_leds = j;
	if (num_leds < 4)
		return false;
		num_leds = 4;
	list_points3d.resize(num_leds);
	list_points2d.resize(num_leds);
	list_points2d_undistorted.resize(num_leds);

	for (j = 0; j < 5/*num_led_pos*/; j++) {
		list_points3d_all[j][0] = leds[j].pos.x/1000000.0;
		list_points3d_all[j][1] = leds[j].pos.y/1000000.0;
		list_points3d_all[j][2] = leds[j].pos.z/1000000.0;
	}

	// we have distortion params for the openCV fisheye model
	// so we undistort the image points manually before passing them to the PnpRansac solver
	// and we give the solver identity camera + null distortion matrices
	cv::fisheye::undistortPoints(list_points2d, list_points2d_undistorted, fishK, fishDistCoeffs);

	// printf("Setting up softposit...\n");

	cv::solvePnPRansac(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec,
			   use_extrinsic_guess, iterationsCount, reprojectionError,
			   confidence, inliers, flags);
				 
	std::vector<Object*> objects(1);
	objects[0] = softposit_new_object(list_points3d_all);

	softposit(objects, list_points2d_undistorted);

	printf("PnPRansac rot: %f %f %f\n", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
	printf("PnPRansac trans: %f %f %f\n", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

	// cv::Rodrigues(objects[0]->rotation, rvec);
	printf("softposit rot: %f %f %f\n", objects[0]->rotation.at<double>(0, 0), objects[0]->rotation.at<double>(0, 1), objects[0]->rotation.at<double>(0, 2));
	printf("               %f %f %f\n", objects[0]->rotation.at<double>(1, 0), objects[0]->rotation.at<double>(1, 1), objects[0]->rotation.at<double>(1, 2));
	printf("               %f %f %f\n", objects[0]->rotation.at<double>(2, 0), objects[0]->rotation.at<double>(2, 1), objects[0]->rotation.at<double>(2, 2));

	tvec = objects[0]->translation;
	printf("softposit trans: %f %f %f %f\n", objects[0]->translation[0], objects[0]->translation[1], objects[0]->translation[2], objects[0]->translation[3]);

	softposit_free_object(objects[0]);

	abort();

	vec3f v;
	double angle = sqrt(rvec.dot(rvec));
	double inorm = 1.0f / angle;

	v.x = rvec.at<double>(0) * inorm;
	v.y = rvec.at<double>(1) * inorm;
	v.z = rvec.at<double>(2) * inorm;
	oquatf_init_axis (rot, &v, angle);

	for (i = 0; i < 3; i++)
		trans->arr[i] = tvec.at<double>(i);


	// LOGV ("Got PnP pose quat %f %f %f %f  pos %f %f %f  leds %u",
	//      rot->x, rot->y, rot->z, rot->w,
	//      trans->x, trans->y, trans->z,
	// 	 num_leds);
	return true;
}

void rift_project_points(rift_led *leds, int num_led_pos,
    dmat3 *camera_matrix, double dist_coeffs[4],
    quatf *rot, vec3f *trans, vec3f *out_points)
{
	cv::Mat fishK = cv::Mat(3, 3, CV_64FC1, camera_matrix->m);
	cv::Mat fishDistCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);

	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  int i;

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	cv::Rodrigues(R, rvec);

	std::vector<cv::Point3f> led_points3d(num_led_pos);
	std::vector<cv::Point2f> projected_points2d(num_led_pos);

	for (int i = 0; i < num_led_pos; i++) {
		led_points3d[i].x = leds[i].pos.x;
		led_points3d[i].y = leds[i].pos.y;
		led_points3d[i].z = leds[i].pos.z;
	}

	cv::fisheye::projectPoints (led_points3d, projected_points2d, rvec, tvec, fishK, fishDistCoeffs);

	for (int i = 0; i < num_led_pos; i++) {
		out_points[i].x = projected_points2d[i].x;
		out_points[i].y = projected_points2d[i].y;
		out_points[i].z = 0;
	}
}
