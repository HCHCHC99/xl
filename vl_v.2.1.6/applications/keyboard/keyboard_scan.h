/**
 * @file    keyboard_scan.h
 * @brief   L1 采集层：CAN 报文 → 按键位掩码（纯函数，无状态）
 */

#ifndef __KEYBOARD_SCAN_H
#define __KEYBOARD_SCAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 解析 0x18FF8380 面板报文为按键掩码
 * @param data 报文数据（8 字节）
 * @param len  长度，必须 >= 8
 * @return KB_* 位掩码；参数非法返回 0
 * @note  纯函数，可在任意上下文调用，便于单元测试
 */
uint16_t kb_scan_parse(const uint8_t *data, uint8_t len);

/**
 * @brief 掩码 → 可读字符串（调试用，静态缓冲区，非重入）
 */
const char *kb_scan_mask_str(uint16_t mask);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_SCAN_H */
