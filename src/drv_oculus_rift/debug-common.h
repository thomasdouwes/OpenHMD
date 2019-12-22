#ifndef __DEBUG_COMMON__
#define __DEBUG_COMMON__

#ifdef __cplusplus
extern "C" {
#endif

#include "../omath.h"

typedef enum {
  DEBUG_POSE_INITIAL,
  DEBUG_POSE_INTERMEDIATE,
  DEBUG_POSE_BEST,
  DEBUG_POSE_FINAL,
} DebugPoseType;

typedef void (*DebugVisCallback) (void *cb_data, DebugPoseType pose_type, quatf *pose_orient, vec3f *trans);

#ifdef __cplusplus
}
#endif

#endif
