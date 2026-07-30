/* OLED Radar UI - Vivid Display V2
 * All params explicit names + units, NO cryptic abbrevs
 * 128x64 SSD1306
 */
#include "Hardware/OLED.h"
#include <math.h>
#include <stdio.h>   /* snprintf() */
#include <string.h>  /* for safety */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static char ui_buf[48];  /* internal snprintf buffer */

/* Draw vivid radar labels: 1M/2M/3M circles + 0/90/180 angles + [OK] status
 * Call AFTER drawing semicircles + reference lines but BEFORE OLED_Refresh()
 */
void OLED_RadarUI_DrawLabels(I2C_HandleTypeDef *hi2c,
                             float servo_conf,
                             float vib_level)
{
    /* -------- Distance labels on circles: user no guess which radius -------- */
    OLED_ShowString(hi2c, 98, 24, "3M");   /* outer 3m circle, upper-right */
    OLED_ShowString(hi2c, 86, 30, "2M");   /* middle 2m */
    OLED_ShowString(hi2c, 74, 34, "1M");   /* inner 1m near origin */

    /* -------- Angle labels on reference lines -------- */
    OLED_ShowString(hi2c, 0,   32, "180"); /* left  = 180 degrees */
    OLED_ShowString(hi2c, 61,  0,  "90");  /* up    = 90 degrees (front) */
    OLED_ShowString(hi2c, 108, 32, "0");   /* right = 0 degrees */

    /* -------- Top-right status icon [OK]/[ ~]/[ !] instant credibility -------- */
    float q = servo_conf;
    if (vib_level > 1.0f) q *= (1.0f - 0.1f * (vib_level - 1.0f));
    if (q < 0.05f) q = 0.05f;
    if      (q >= 0.80f) OLED_ShowString(hi2c, 104, 0, "[OK]");   /* stable, trust */
    else if (q >= 0.40f) OLED_ShowString(hi2c, 104, 0, "[ ~]");   /* so-so */
    else                 OLED_ShowString(hi2c, 104, 0, "[ !]");   /* vibrating/bad */
}

/* Draw obstacle point with size by distance + confidence
 *  <0.5m + high conf => 3x3 big square (DANGER, stand out!)
 *  0.5~2m            => cross +
 *  >2m or low conf   => single dot . (far or low trust, no scare user)
 */
void OLED_RadarUI_DrawObstacle(I2C_HandleTypeDef *hi2c,
                               int16_t px, int16_t py,
                               uint16_t dist_mm, float quality)
{
    /* ---------- 关键Fix1：放宽边界，别把近距离/边缘的障碍物直接丢了 ---------- */
    if (px < 0) px = 0;                /* 允许贴左边界（0°） */
    if (px > 127) px = 127;            /* 允许贴右边界（180°） */
    if (py < 0) py = 0;                /* 允许贴顶（正前方90°） */
    if (py > 41) py = 41;              /* 允许贴圆心线下方1px（<10cm极近距离） */
    /* ---------- 关键Fix2：放宽分级阈值，原来0.8太严，稳不住 ---------- */
    if (dist_mm < 500 && quality >= 0.60f) {
        /* 近距离 + 中高可信度 → 3x3实心方块(危险警示) */
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px-1, (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px+1, (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py-1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py+1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px-1, (uint8_t)py-1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px+1, (uint8_t)py-1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px-1, (uint8_t)py+1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px+1, (uint8_t)py+1, 1);
    } else if (dist_mm <= 2000 || quality >= 0.25f) {
        /* 0.5~2m 中距离 或 可信度尚可 → 十字点（只要可信度>0.25，哪怕远也给十字，容易看到） */
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px-1, (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px+1, (uint8_t)py,   1);
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py-1, 1);
        OLED_DrawPoint(hi2c, (uint8_t)px,   (uint8_t)py+1, 1);
    } else {
        /* 超远或可信度极低 → 至少留个单点提示，别直接空 */
        OLED_DrawPoint(hi2c, (uint8_t)px, (uint8_t)py, 1);
    }
}

/* Line 1 (y=40): D1=Scan D2=BreadBoard A=Angle (all explicit prefix) */
void OLED_RadarUI_Line1_DistAngle(I2C_HandleTypeDef *hi2c,
                                  uint16_t d1, uint16_t d2,
                                  uint16_t angle, uint16_t invalid)
{
    if (d1 == invalid) d1 = 9999;
    if (d2 == invalid) d2 = 9999;
    snprintf(ui_buf, sizeof(ui_buf),
             "D1:%4d D2:%4d A%3d", d1, d2, angle);
    OLED_ShowString(hi2c, 0, 40, ui_buf);
}

/* Line 2 (y=48): AIR temp (Celsius) + RH humidity (percent)
 * prefix "?" if not calibrated yet
 */
void OLED_RadarUI_Line2_Weather(I2C_HandleTypeDef *hi2c,
                                float air_temp_c, float rh_pct,
                                uint8_t calibrated)
{
    const char *qm = calibrated ? "" : "?";
    snprintf(ui_buf, sizeof(ui_buf),
             "%sAIR%3.1fC %sRH%2.0f%%",
             qm, air_temp_c, qm, rh_pct);
    OLED_ShowString(hi2c, 0, 48, ui_buf);
}

/* Line 3 (y=56): Q=Quality(0-100 data quality score)
 *               V=Vibration(Degree Per Second, gyro standard unit)
 * Removed cryptic TM/C/% - replaced by intuitive Q score + gyro DPS unit
 */
void OLED_RadarUI_Line3_MechStatus(I2C_HandleTypeDef *hi2c,
                                   float servo_conf, float vib_dps)
{
    float vc = 1.0f;
    if (vib_dps > 1.0f) {
        vc = 1.0f - 0.1f * (vib_dps - 1.0f);
        if (vc < 0.1f) vc = 0.1f;
    }
    float Q = servo_conf * vc * 100.0f;
    if (Q < 5.0f) Q = 5.0f;
    float V = vib_dps;
    if (V > 9.9f) V = 9.9f;
    snprintf(ui_buf, sizeof(ui_buf), "Q%2.0f V%1.1fD/S", Q, V);
    OLED_ShowString(hi2c, 0, 56, ui_buf);
}