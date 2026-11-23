/**
 * @file    seq_arbit.c
 * @brief   一键序列专属仲裁规则（P3）—— 纯函数实现
 * @note    零 RTOS 依赖：不 include rtthread.h、不读硬件、无静态状态。
 */

#include "seq_arbit.h"

/* ============================================================
 * 一键仲裁
 * ============================================================ */
seq_arb_t seq_arbit_check_onekey(seq_type_t type, int32_t swing_x10, uint8_t key_count)
{
    /* ---- 1. 同拍多键：任意组合都不启动（TC-PRI-002）----
     * 放在最前：三个键同按时，"右后可能超限"之类的判断没有意义，
     * 规矩是"一个都不启动"。 */
    if (key_count >= 2U) {
        return SEQ_ARB_DENY_MULTI_KEY;
    }

    /* ---- 2. 超右限位（> +217°）：只禁"右后" ----
     * 右中（目标 +120°）与回位（目标 0°）都是从当前位置向内转，放行。
     * 用例明确写"超右时右后无输出"，即使 +218°→+215° 是向内左转也禁止。 */
    if (swing_x10 > SEQ_ARB_RIGHT_GUARD_X10) {
        if (type == SEQ_TYPE_RIGHT_BEHIND) {
            return SEQ_ARB_DENY_LIMIT_RIGHT;
        }
    }

    /* ---- 3. 超左限位（< −102°）：三种一键均可执行（TC-ANG-012 / PRI-009）----
     * 用例只要求"左转无输出"，而左转属手动，由 swing 的 forbidden_dirs 拦截；
     * 一键侧不设禁止项。此处保留门槛判断分支，便于将来若客户收紧规则时扩展。 */
    if (swing_x10 < SEQ_ARB_LEFT_GUARD_X10) {
        /* 明确：无禁止项 */
    }

    return SEQ_ARB_OK;
}

/* ============================================================
 * 原因文本
 * ============================================================ */
const char *seq_arbit_reason(seq_arb_t r)
{
    switch (r) {
        case SEQ_ARB_OK:                 return "OK";
        case SEQ_ARB_DENY_MULTI_KEY:     return "multi-key same frame";
        case SEQ_ARB_DENY_LIMIT_RIGHT:   return "over right limit (>+217.0)";
        case SEQ_ARB_DENY_ANGLE_INVALID: return "angle invalid";
        default:                         return "unknown";
    }
}