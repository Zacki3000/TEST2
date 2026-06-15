#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void gnss_pps_init(void);
void gnss_pps_push_gnss_time(int64_t gnss_time_unix_seconds);

#ifdef __cplusplus
}
#endif