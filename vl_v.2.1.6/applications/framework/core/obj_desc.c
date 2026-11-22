/**
 * @file    obj_desc.c
 * @brief   被控对象描述表实现（2026-11-15 C1 批：字段定型 + 填表）
 * @note    零 RTOS 依赖。表数据全部位于 .rodata（Flash），不占 RAM。
 *
 *          历史：PLAN-OBJ-BASE-V1.0 只建表结构与空表（G-07），
 *                LEG-3 的 app_main 路由一直未接线。
 *
 *          本批（Q-C1 / Q-C2 / Q-C5 / Q-C8 按建议代裁定）：
 *            ①结构补 resources / layer 两个字段（具名 tag 供 obj_base.h 前置声明）；
 *            ②按 Q-C5 建议**先填 swing / lift 两行**；
 *              clutch / calib 枚举有值但模块未建成 ⇒ 不占位（避免假表项）；
 *              motor 枚举无值 ⇒ 不注册（R-12 已纠正设计基线 §2.5）；
 *            ③event_mask 按 Q-C8 建议收敛为「0 = 全收本区间；非 0 = 仅 offset 0~31」，
 *              不扩成 4096 位图；
 *            ④app_main 的查表接线属 C2 批，本文件不改路由。
 *
 *          ⚠️ 能力边界（Q-C7）：描述表只承载**静态描述**
 *          （事件域 / 能力 / 依赖 / 优先级 / 分层）。按键优先级全序、一键互斥、
 *          超限降级等带记忆的时序语义不在此表，另立批次。
 *          论证见 `docs/02-设计/方案-被控对象设计-V1.0.md` 第 II 部分。
 */

#include "obj_desc.h"
#include "app_event.h"                             /* LIFT/SWING_EVENT_BASE */
#include "mode/sys_caps.h"                         /* SYS_CAP_MANUAL */
#include "framework/state_engine/state_engine.h"   /* StateMachineConfig_t */
#include <stddef.h>

/* 业务侧状态机配置（已存在，描述表只做引用，不产生第二份真值） */
extern const StateMachineConfig_t g_lift_config;
extern const StateMachineConfig_t g_swing_config;

/* ============================================================
 * 描述表（C1：2 行）
 * @note  结构体 44 B/条由 obj_desc.h 的编译期 size_check 实测；
 *        .rodata 增量以编译前后 arm-none-eabi-size 差值为准（R-06）。
 * ============================================================ */
const ctrl_obj_desc_t g_ctrl_obj_descs[] = {
    {
        .type          = CTRL_OBJ_TYPE_LIFT,
        .name          = "LiftControl",      /* 与 lift_control.c 注册名一致 */
        .event_base    = LIFT_EVENT_BASE,    /* 0x1000 */
        .event_mask    = 0U,                 /* Q-C8：0 = 全收 0x1000~0x1FFF */
        .priority      = 0U,                 /* REQ-014 仲裁规则未定稿 ⇒ 0 = 未启用 */
        .required_caps = SYS_CAP_MANUAL,     /* Q-C4：能力门禁接现有 sys_caps_get() */
        .depends_on    = NULL,               /* 依赖列表暂为空 */
        .sm_cfg        = &g_lift_config,
        .obj_cfg       = NULL,               /* 限位配置未对象化（B-3 / D-P6 待办）*/
        .ops           = NULL,               /* LEG-6：虚表未接线 */
        .resources     = NULL,               /* Q-C1：字段先行，绑定规则待定义 */
        .layer         = CTRL_OBJ_LAYER_CONTROLLED,
    },
    {
        .type          = CTRL_OBJ_TYPE_SWING,
        .name          = "SwingControl",     /* 与 swing_control.c 注册名一致 */
        .event_base    = SWING_EVENT_BASE,   /* 0x2000 */
        .event_mask    = 0U,                 /* Q-C8：0 = 全收 0x2000~0x2FFF */
        .priority      = 0U,
        .required_caps = SYS_CAP_MANUAL,
        .depends_on    = NULL,
        .sm_cfg        = &g_swing_config,
        .obj_cfg       = NULL,
        .ops           = NULL,
        .resources     = NULL,
        .layer         = CTRL_OBJ_LAYER_CONTROLLED,
    },
};

const uint8_t g_ctrl_obj_desc_count =
    (uint8_t)(sizeof(g_ctrl_obj_descs) / sizeof(g_ctrl_obj_descs[0]));

/* ============================================================
 * 按类型查描述
 * @note  obj_base_init() 在初始化时调用本函数，把描述指针绑定到 obj->desc。
 *        未填表项的对象（mode / seq / vehicle）返回 NULL，属预期行为。
 * ============================================================ */
const ctrl_obj_desc_t *obj_desc_find_by_type(uint32_t type)
{
    uint8_t i;

    for (i = 0; i < g_ctrl_obj_desc_count; i++) {
        if (g_ctrl_obj_descs[i].type == type) {
            return &g_ctrl_obj_descs[i];
        }
    }
    return NULL;
}

/* ============================================================
 * 按事件号查归属对象
 * @note  这是替代 app_main.c 那段 if-else 区间路由的目标接口（LEG-3）。
 *
 *        【Q-C8 定稿语义，2026-11-15】
 *        - event_mask == 0：该对象接收 [event_base, event_base + 0x1000) 全部事件
 *        - event_mask != 0：位掩码**只覆盖 offset 0~31**；offset >= 32 一律不命中。
 *          因为 app_event.h 每个模块占 0x1000 区间，想覆盖高位事件只能
 *          置 0 全收，或另设 event_base 分段。禁止把 mask 当 4096 位图使用。
 *        - 区间宽度固定 0x1000，与 app_event.h 的模块区间划分一致。
 *
 *        历史问题：原注释写「位掩码，允许非连续事件」，但实现只对 offset<32
 *        有效，与 0x1000 区间不匹配 ⇒ 属实现与已定设计冲突（R-07）。
 *        按 Q-C8 建议**保留实现、把语义写进规范**，不改判定逻辑。
 * ============================================================ */
const ctrl_obj_desc_t *obj_desc_find_by_event(uint32_t event)
{
    uint8_t  i;
    uint32_t offset;

    for (i = 0; i < g_ctrl_obj_desc_count; i++) {
        const ctrl_obj_desc_t *d = &g_ctrl_obj_descs[i];

        /* 事件落在该对象区间内？ */
        if (event < d->event_base) {
            continue;
        }

        offset = event - d->event_base;

        /* offset 必须落在 0x1000 步长的区间内（与现状基址间距一致） */
        if (offset >= 0x1000u) {
            continue;
        }

        /* Q-C8：mask 为 0 全收；非 0 只认 32 位窗口内的位 */
        if (d->event_mask == 0u) {
            return d;
        }
        if (offset < 32u && (d->event_mask & (1u << offset)) != 0u) {
            return d;
        }
    }
    return NULL;
}
