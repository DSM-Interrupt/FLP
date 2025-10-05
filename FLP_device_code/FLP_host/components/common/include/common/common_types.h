/*
 * common_type.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

#define MAX_DEVICE_CNT 50

typedef struct {
    int Code;
    double Latit;
    double Longit;
    uint8_t Danger;
} TerminalDevice;

#endif // COMMON_TYPES_H