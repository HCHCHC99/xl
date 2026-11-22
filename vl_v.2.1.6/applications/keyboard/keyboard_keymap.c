/**
 * @file    keyboard_keymap.c
 * @brief   L2 键值层实现：键值映射表
 *
 * 【职责】把按键掩码归类为「有名字的键值」，用于调试显示与统计。
 *
 * 【重要】本层输出**不决定执行行为**。
 *   互斥（同轴对向）的判定与消解由 keyboard_sm.c 的 _kb_axis_filter()
 *   按「每轴独立 + 300ms 窗口 + 状态帧语义」完成，因为需要跨帧记忆，
 *   单帧纯函数无法表达。
 *   例如 `上+下+左`：本层归为 MULTI，但实际效果是「举升轴视为没按、
 *   回转轴左转生效」——这一结果只能由 _kb_axis_filter 逐轴得出。
 *
 * 【扩展方式】新增组合键只需在 s_keymap[] 加一行，解析逻辑无需改动。
 */

#include "keyboard_keymap.h"

/* ============================================================
 * 键值映射表
 * ============================================================ */
typedef struct {
    const char *name;
    uint16_t    mask;       /* 仅含方向位/一键位的精确掩码 */
    key_code_t  code;
} kb_keymap_entry_t;

static const kb_keymap_entry_t s_keymap[] = {
    /* ----- 单键 ----- */
    { "UP",          KB_UP,                          KEY_CODE_UP          },
    { "DOWN",        KB_DOWN,                        KEY_CODE_DOWN        },
    { "LEFT",        KB_LEFT,                        KEY_CODE_LEFT        },
    { "RIGHT",       KB_RIGHT,                       KEY_CODE_RIGHT       },
    { "ONKEY_MID",   KB_ONKEY_MID,                   KEY_CODE_ONKEY_MID   },
    { "ONKEY_REAR",  KB_ONKEY_REAR,                  KEY_CODE_ONKEY_REAR  },
    { "ONKEY_HOME",  KB_ONKEY_HOME,                  KEY_CODE_ONKEY_HOME  },

    /* ----- 同轴对向（仅用于显示归类，实际消解见 _kb_axis_filter） ----- */
    { "UP_DOWN",     KB_UP | KB_DOWN,                KEY_CODE_UP_DOWN     },
    { "LEFT_RIGHT",  KB_LEFT | KB_RIGHT,             KEY_CODE_LEFT_RIGHT  },

    /* ----- 异轴双键（合法，两轴同时动作） ----- */
    { "UP_LEFT",     KB_UP | KB_LEFT,                KEY_CODE_UP_LEFT     },
    { "UP_RIGHT",    KB_UP | KB_RIGHT,               KEY_CODE_UP_RIGHT    },
    { "DOWN_LEFT",   KB_DOWN | KB_LEFT,              KEY_CODE_DOWN_LEFT   },
    { "DOWN_RIGHT",  KB_DOWN | KB_RIGHT,             KEY_CODE_DOWN_RIGHT  },
};

#define KEYMAP_COUNT    (sizeof(s_keymap) / sizeof(s_keymap[0]))

/* 参与键值判定的掩码（校准/离合位不参与） */
#define KEYMAP_RELEVANT (KB_MANUAL_MASK | KB_ONEKEY_MASK)

/* ============================================================
 * 内部工具
 * ============================================================ */
static uint8_t popcount16(uint16_t v)
{
    uint8_t n = 0U;

    while (v != 0U) {
        n = (uint8_t)(n + (v & 1U));
        v = (uint16_t)(v >> 1);
    }
    return n;
}

/* ============================================================
 * 解析
 * ============================================================ */
key_code_t kb_keymap_resolve(uint16_t mask)
{
    uint16_t m = (uint16_t)(mask & KEYMAP_RELEVANT);
    uint16_t i;

    if (m == 0U) {
        return KEY_CODE_NONE;
    }

    /* 1. 精确匹配 */
    for (i = 0U; i < KEYMAP_COUNT; i++) {
        if (s_keymap[i].mask == m) {
            return s_keymap[i].code;
        }
    }

    /* 2. 手动 + 一键 类别冲突 */
    if (((m & KB_MANUAL_MASK) != 0U) && ((m & KB_ONEKEY_MASK) != 0U)) {
        return KEY_CODE_MANUAL_ONEKEY;
    }

    /* 3. 三键及以上（全部忽略） */
    if (popcount16(m) >= 3U) {
        return KEY_CODE_MULTI;
    }

    /* 4. 其余组合（如 一键 + 一键、单键之外的未定义组合）→ 忽略 */
    return KEY_CODE_NONE;
}

const char *kb_key_code_name(key_code_t k)
{
    /* 先查表（单键/双键名与表一致） */
    switch (k) {
    case KEY_CODE_NONE:          return "NONE";
    case KEY_CODE_UP:            return "UP";
    case KEY_CODE_DOWN:          return "DOWN";
    case KEY_CODE_LEFT:          return "LEFT";
    case KEY_CODE_RIGHT:         return "RIGHT";
    case KEY_CODE_ONKEY_MID:     return "ONKEY_MID";
    case KEY_CODE_ONKEY_REAR:    return "ONKEY_REAR";
    case KEY_CODE_ONKEY_HOME:    return "ONKEY_HOME";
    case KEY_CODE_CALIB:         return "CALIB";
    case KEY_CODE_UP_DOWN:       return "UP_DOWN";
    case KEY_CODE_LEFT_RIGHT:    return "LEFT_RIGHT";
    case KEY_CODE_UP_LEFT:       return "UP_LEFT";
    case KEY_CODE_UP_RIGHT:      return "UP_RIGHT";
    case KEY_CODE_DOWN_LEFT:     return "DOWN_LEFT";
    case KEY_CODE_DOWN_RIGHT:    return "DOWN_RIGHT";
    case KEY_CODE_MANUAL_ONEKEY: return "MANUAL_ONEKEY";
    case KEY_CODE_MULTI:         return "MULTI";
    default:                     return "?";
    }
}

uint16_t kb_keymap_entry_count(void)
{
    return (uint16_t)KEYMAP_COUNT;
}
