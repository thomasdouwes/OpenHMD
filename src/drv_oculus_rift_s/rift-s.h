/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift S Driver Internal Interface */
#ifndef RIFT_S_H
#define RIFT_S_H

#include "../openhmdi.h"

typedef struct rift_s_hmd_s rift_s_hmd_t;
typedef struct rift_s_device_priv_s rift_s_device_priv;

struct rift_s_device_priv_s {
	ohmd_device base;
	int id;
	bool opened;

	rift_s_hmd_t *hmd;
};

#endif
