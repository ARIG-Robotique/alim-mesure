//
// Created by gregorydepuille@sglk.local on 13/03/2021.
//

#ifndef ALIM_MESURE_ITM_LOG_H
#define ALIM_MESURE_ITM_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

#define LOG_DEBUG_LEVEL 24
#define LOG_INFO_LEVEL 25
#define LOG_WARN_LEVEL 26
#define LOG_ERROR_LEVEL 27

#define LOG_LEVEL_MIN LOG_WARN_LEVEL

#define LOG_DEBUG(msg) println(msg, LOG_DEBUG_LEVEL)
#define LOG_INFO(msg) println(msg, LOG_INFO_LEVEL)
#define LOG_WARN(msg) println(msg, LOG_WARN_LEVEL)
#define LOG_ERROR(msg) println(msg, LOG_ERROR_LEVEL)

void print(char _char, uint8_t level);

void println(const char *msg, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif //ALIM_MESURE_ITM_LOG_H
