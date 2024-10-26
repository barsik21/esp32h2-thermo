#include <stdint.h>
#include <stdbool.h>

#define T_STATUS_FAILED "Failed!"
#define T_STATUS_DONE "Done"

//char strftime_buf[64];


void get_rtc_time();
bool int_to_bool(int32_t value);

void setup_NVS();
int16_t read_NVS(const char *nvs_key);
bool write_NVS(const char *nvs_key, int value);