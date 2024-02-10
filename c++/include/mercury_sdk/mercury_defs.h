#ifndef MERCURY_SDK_INCLUDE_MERCURY_SDK_MERCURY_DEFS_H_
#define MERCURY_SDK_INCLUDE_MERCURY_SDK_MERCURY_DEFS_H_

#include <stdint.h>

struct mcy_servo
{
  uint8_t id;
  int32_t mcy_present_position;
};

#endif /* MERCURY_SDK_INCLUDE_MERCURY_SDK_MERCURY_DEFS_H_ */