/**
 * @file    keyboard_keymap.h
 * @brief   L2 键值层：按键掩码 → 虚拟键值（查表，纯函数）
 */

#ifndef __KEYBOARD_KEYMAP_H
#define __KEYBOARD_KEYMAP_H

#include <stdint.h>
#include "keyboard_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 掩码 → 虚拟键值
 * @param mask 按键位掩码（KB_* 组合）
 * @return 键值
 * @note  只看方向位与一键位；校准位与离合位由上层单独处理，不参与键值判定。
 *        解析优先级：精确匹配 → 手动+一键 → 多键(>=3) → NONE
 */
key_code_t kb_keymap_resolve(uint16_t mask);

/**
 * @brief 键值名（调试用）
 */
const char *kb_key_code_name(key_code_t k);

/**
 * @brief 键值表条目数（调试/自检用）
 */
uint16_t kb_keymap_entry_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_KEYMAP_H */
