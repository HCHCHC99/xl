/**
 * @file    keyboard_scan.c
 * @brief   L1 采集层实现：报文位域 → 按键掩码
 */

#include "keyboard_scan.h"
#include "keyboard_defs.h"

#include "../CAN/protocol_defs.h"

#include <stdio.h>

uint16_t kb_scan_parse(const uint8_t *data, uint8_t len)
{
    const panel_switch_t *p;
    uint16_t mask = 0U;

    if ((data == RT_NULL) || (len < 8U)) {
        return 0U;
    }

    p = (const panel_switch_t *)data;

    /* Byte1：上 / 下 / 左 / 右（各位 2bit，01 = 按下） */
    if (PANEL_KEY_PRESSED(p->bits.key_up) != 0U)    { mask |= KB_UP;    }
    if (PANEL_KEY_PRESSED(p->bits.key_down) != 0U)  { mask |= KB_DOWN;  }
    if (PANEL_KEY_PRESSED(p->bits.key_left) != 0U)  { mask |= KB_LEFT;  }
    if (PANEL_KEY_PRESSED(p->bits.key_right) != 0U) { mask |= KB_RIGHT; }

    /* Byte2：一键回位 / 校准 / 一键右中 / 一键右后 */
    if (PANEL_KEY_PRESSED(p->bits.onekey_home) != 0U) { mask |= KB_ONKEY_HOME; }
    if (PANEL_KEY_PRESSED(p->bits.key_calib) != 0U)   { mask |= KB_CALIB;      }
    if (PANEL_KEY_PRESSED(p->bits.onekey_mid) != 0U)  { mask |= KB_ONKEY_MID;  }
    if (PANEL_KEY_PRESSED(p->bits.onekey_rear) != 0U) { mask |= KB_ONKEY_REAR; }

    /* Byte0：离合（预留，暂不参与判定） */
    if (PANEL_KEY_PRESSED(p->bits.clutch_combine) != 0U)  { mask |= KB_CLUTCH_COM; }
    if (PANEL_KEY_PRESSED(p->bits.clutch_separate) != 0U) { mask |= KB_CLUTCH_SEP; }

    return mask;
}

const char *kb_scan_mask_str(uint16_t mask)
{
    static char buf[96];
    int n = 0;

    buf[0] = '\0';

    if (mask == 0U) {
        (void)snprintf(buf, sizeof(buf), "NONE");
        return buf;
    }

    if ((mask & KB_UP) != 0U)         { n += snprintf(buf + n, sizeof(buf) - n, "UP "); }
    if ((mask & KB_DOWN) != 0U)       { n += snprintf(buf + n, sizeof(buf) - n, "DOWN "); }
    if ((mask & KB_LEFT) != 0U)       { n += snprintf(buf + n, sizeof(buf) - n, "LEFT "); }
    if ((mask & KB_RIGHT) != 0U)      { n += snprintf(buf + n, sizeof(buf) - n, "RIGHT "); }
    if ((mask & KB_ONKEY_MID) != 0U)  { n += snprintf(buf + n, sizeof(buf) - n, "OKMID "); }
    if ((mask & KB_ONKEY_REAR) != 0U) { n += snprintf(buf + n, sizeof(buf) - n, "OKREAR "); }
    if ((mask & KB_ONKEY_HOME) != 0U) { n += snprintf(buf + n, sizeof(buf) - n, "OKHOME "); }
    if ((mask & KB_CALIB) != 0U)      { n += snprintf(buf + n, sizeof(buf) - n, "CALIB "); }
    if ((mask & KB_CLUTCH_COM) != 0U) { n += snprintf(buf + n, sizeof(buf) - n, "CLT_COM "); }
    if ((mask & KB_CLUTCH_SEP) != 0U) { n += snprintf(buf + n, sizeof(buf) - n, "CLT_SEP "); }

    return buf;
}
