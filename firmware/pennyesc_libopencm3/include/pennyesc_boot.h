#ifndef PENNYESC_BOOT_H
#define PENNYESC_BOOT_H

#include <stdint.h>

void pennyesc_boot_request(uint32_t now_ms);
void pennyesc_boot_poll(uint32_t now_ms);

#endif
