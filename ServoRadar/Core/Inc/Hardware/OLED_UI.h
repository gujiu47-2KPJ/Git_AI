#ifndef __OLED_UI_H__
#define __OLED_UI_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ==============================================================
 * OLED Radar Vivid UI V2
 * 
 * PURPOSE: Eliminate ALL cryptic abbreviations.
 * All displays use EXPLICIT names + UNITS.
 * 
 * OLD (cryptic user-unfriendly):  S1234 B5678 90 / TE28.5 H65% / TM40 V1.5 C90%
 * NEW (explicit + units)       : D1:1234 D2:5678 A90 / AIR28.5C RH65% / Q90 V1.5D/S
 * Also adds: 1M/2M/3M/0/90/180 numeric labels on radar + [OK] status icon.
 * Also adds: graded obstacle dots (close+trusted=big square, mid=cross, far=dot)
 * ============================================================== */

/* Draw radar skeleton labels + top-right status icon.
 * Call AFTER:  OLED_Clear() + semicircles + reference lines drawn
 * Call BEFORE: scan line, obstacle dots */
void OLED_RadarUI_DrawLabels(I2C_HandleTypeDef *hi2c,
                             float servo_confidence,  /* 0.0~1.0 */
                             float vibration_dps);    /* dual-MPU diff deg/s */

/* Draw obstacle with size by distance + quality (graded 3 levels) */
void OLED_RadarUI_DrawObstacle(I2C_HandleTypeDef *hi2c,
                               int16_t px, int16_t py,
                               uint16_t dist_mm,
                               float total_quality);   /* 0.0~1.0 */

/* ------- 3 data lines (y=40 / 48 / 56) ------- */
/* L1: D1=<scan:mm> D2=<breadboard:mm> A=<angle:deg> */
void OLED_RadarUI_Line1_DistAngle(I2C_HandleTypeDef *hi2c,
                                  uint16_t dist_scan_mm,
                                  uint16_t dist_base_mm,
                                  uint16_t servo_angle_deg,
                                  uint16_t dist_invalid_val);

/* L2: AIR=<fused ambient temp used for speed of sound>C RH=<humidity>%
 *     prefix "?" when NOT yet calibrated via ESP32 packet */
void OLED_RadarUI_Line2_Weather(I2C_HandleTypeDef *hi2c,
                                float air_temp_celsius,
                                float humidity_percent,
                                uint8_t calibrated_flag);

/* L3: Q=<data quality 0..100> V=<vibration DPS>  (DPS=degree/sec gyro std unit)*/
void OLED_RadarUI_Line3_MechStatus(I2C_HandleTypeDef *hi2c,
                                   float servo_confidence,
                                   float vibration_dps);

#endif /* __OLED_UI_H__ */