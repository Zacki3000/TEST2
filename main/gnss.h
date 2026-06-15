#pragma once

#include "RustApi.h"

#ifdef __cplusplus
extern "C" {
#endif

void gnss_init(void);
GnssState *get_gnss_state();

#ifdef __cplusplus
}
#endif