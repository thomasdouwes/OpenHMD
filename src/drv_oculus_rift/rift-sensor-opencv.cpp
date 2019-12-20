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
	int num_matched = 0;
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

	if (num_blobs < 4) {
		printf("Not enough blobs: %d\n", num_blobs);
		return false;
	}

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	cv::Rodrigues(R, rvec);

	//cout << "R = " << R << ", rvec = " << rvec << endl;
	//cout << "fishK = " << fishK << " dist " << fishDistCoeffs << endl;

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

	// if (num_leds < 4)
	// 	return false;

	std::vector<cv::Vec3d> list_points3d(num_leds);
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

		printf ("LED %d at %d,%d (3D %f %f %f)\n",
		    blobs[i].led_id, blobs[i].x, blobs[i].y,
		    leds[blobs[i].led_id].pos.x,
		    leds[blobs[i].led_id].pos.y,
		    leds[blobs[i].led_id].pos.z);
	}

	num_matched = j;
	if (num_matched >= 4) {
			list_points3d.resize(num_matched);
			list_points2d.resize(num_matched);
			list_points2d_undistorted.resize(num_matched);
		
			// we have distortion params for the openCV fisheye model
			// so we undistort the image points manually before passing them to the PnpRansac solver
			// and we give the solver identity camera + null distortion matrices
			cv::fisheye::undistortPoints(list_points2d, list_points2d_undistorted, fishK, fishDistCoeffs);
		
			// for (i = 0; i < num_blobs; i++) {
			// 	printf("point2d_undistorted %d: %f %f\n", i, list_points2d_all_undistorted[i].x, list_points2d_all_undistorted[i].y);
			// }
			// printf("Setting up softposit...\n");
		
		  cv::solvePnPRansac(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec,
							use_extrinsic_guess, iterationsCount, reprojectionError,
							confidence, inliers, flags);
		
			printf("PnPRansac rot: %f %f %f\n", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
			printf("PnPRansac trans: %f %f %f\n", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
		
			vec3f v;
			double angle = sqrt(rvec.dot(rvec));
			double inorm = 1.0f / angle;
		
			v.x = rvec.at<double>(0) * inorm;
			v.y = rvec.at<double>(1) * inorm;
			v.z = rvec.at<double>(2) * inorm;
			oquatf_init_axis (rot, &v, angle);
		
			for (i = 0; i < 3; i++)
				trans->arr[i] = tvec.at<double>(i);
		
		  float rot_mat[4][4];
		  oquatf_get_mat4x4(rot, trans, rot_mat);
			printf("PnPRansac rot: %f %f %f\n", rot_mat[0][0], rot_mat[0][1], rot_mat[0][2]);
			printf("               %f %f %f\n", rot_mat[1][0], rot_mat[1][1], rot_mat[1][2]);
			printf("               %f %f %f\n", rot_mat[2][0], rot_mat[2][1], rot_mat[2][2]);
	}

  if (num_blobs >= 4) {
	/* SoftPOSIT matching */
	std::vector<cv::Vec3d> list_points3d_all(num_led_pos);
	std::vector<cv::Vec3d> list_normals_all(num_led_pos);
	std::vector<cv::Point2f> list_points2d_all(num_blobs);
	std::vector<cv::Point2f> list_points2d_all_undistorted; // (num_blobs);

	for (i = 0; i < num_blobs; i++) {
		list_points2d_all[i].x = blobs[i].x;
		list_points2d_all[i].y = blobs[i].y;
		//printf("point2d %d: %d %d %d %d\n", i, blobs[i].x, blobs[i].y, blobs[i].width, blobs[i].height);
	}
	for (j = 0; j < num_led_pos; j++) {
		list_points3d_all[j][0] = leds[j].pos.x;
		list_points3d_all[j][1] = leds[j].pos.y;
		list_points3d_all[j][2] = leds[j].pos.z;
		list_normals_all[j][0] = leds[j].dir.x;
		list_normals_all[j][1] = leds[j].dir.y;
		list_normals_all[j][2] = leds[j].dir.z;
	}

	cv::fisheye::undistortPoints(list_points2d_all, list_points2d_all_undistorted, fishK, fishDistCoeffs);

	Object *obj = softposit_new_object(list_points3d_all, list_normals_all);
				 
	softposit_data *sp = softposit_new();
	softposit_add_object(sp, obj);

	softposit(sp, list_points2d_all_undistorted);

	cv::Rodrigues(obj->rotation, rvec);
	printf("softposit rot: %f %f %f\n", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
	// printf("softposit rot: %f %f %f\n", obj->rotation.at<double>(0, 0), obj->rotation.at<double>(0, 1), obj->rotation.at<double>(0, 2));
	// printf("               %f %f %f\n", obj->rotation.at<double>(1, 0), obj->rotation.at<double>(1, 1), obj->rotation.at<double>(1, 2));
	// printf("               %f %f %f\n", obj->rotation.at<double>(2, 0), obj->rotation.at<double>(2, 1), obj->rotation.at<double>(2, 2));

	printf("softposit trans: %f %f %f %f\n", obj->translation[0], obj->translation[1], obj->translation[2], obj->translation[3]);

			vec3f v;
			double angle = sqrt(rvec.dot(rvec));
			double inorm = 1.0f / angle;
		
			v.x = rvec.at<double>(0) * inorm;
			v.y = rvec.at<double>(1) * inorm;
			v.z = rvec.at<double>(2) * inorm;
			oquatf_init_axis (rot, &v, angle);
		
			for (i = 0; i < 3; i++)
				trans->arr[i] = obj->translation[i];

	softposit_free_object(obj);
	softposit_free(sp);
  return true;
  }

	// abort();

	// LOGV ("Got PnP pose quat %f %f %f %f  pos %f %f %f  leds %u",
	//      rot->x, rot->y, rot->z, rot->w,
	//      trans->x, trans->y, trans->z,
	// 	 num_leds);
	return (num_matched >= 4);
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
