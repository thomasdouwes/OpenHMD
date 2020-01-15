/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2020 Jan Schmidt
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Unit Tests - Pose Tests */

#include "tests.h"

static const vec3f test_position = {{ -150.984375, -371.769531, 326.843750 }};
static const quatf test_orient = {{ 0.833051, 0.238250, 0.341881, -0.363842 }};

const float epsilon = 0.001;

void test_oposef_init()
{
	posef p;

	oposef_init (&p, &test_position, &test_orient);

	TAssert(vec3f_eq(test_position, p.pos, epsilon));
	TAssert(quatf_eq(test_orient, p.orient, epsilon));
}

void test_oposef_inverse()
{
	posef p1, p2, p3;

	oposef_init (&p1, &test_position, &test_orient);

	//printf("Pose:\n");
	//printf("%f %f %f\n", p1.pos.x, p1.pos.y, p1.pos.z); 
	//printf("%f %f %f %f\n", p1.orient.x, p1.orient.y, p1.orient.z, p1.orient.w);

  p2 = p1;
	oposef_inverse (&p2);

	//printf("Inverse:\n");
	//printf("%f %f %f\n", p2.pos.x, p2.pos.y, p2.pos.z); 
	//printf("%f %f %f %f\n", p2.orient.x, p2.orient.y, p2.orient.z, p2.orient.w);

	oposef_inverse (&p2);

	TAssert(vec3f_eq(p1.pos, p2.pos, epsilon));
	TAssert(quatf_eq(p1.orient, p2.orient, epsilon));
}
