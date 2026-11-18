/**
 * @file    obj_desc.h
 * @brief   被控对象描述表（G-07）—— 用数据描述"对象是什么、管什么、要什么"
 * @note    本文件零 RTOS 依赖，纯声明 + 静态表。
 *
 *          背景（架构建模核查 V1.1）：控制对象当前只有 9 个运行时字段，
 *          **不知道自己管哪些事件、要什么权限、依赖谁、限位在哪**。
 *          这导致 app_main.c:199 只能靠事件号区间猜目标对象。
 *          本表就是为了补上这块缺失的"描述数据"。
 *
 *          本批（PLAN-OBJ-BASE-V1.0）**只建表结构与空表**，
 *          内容填充与 app_main 路由改造属 G-07 后续批。
 *
 *          2026-11-15 C1 批（Q-C1/Q-C2/Q-C8 按建议代裁定）：
 *          ①结构补 resources / layer 两个字段，并给出具名 tag 供 obj_base.h 前置声明；
 *          ②表填充 swing / lift 两行；③event_mask 语义按 Q-C8 建议写入规范
 *            （0=全收；非 0 仅覆盖区间内 offset 0~31）。
 *          结构与基类新增字节一律**编译期实测**（见文末 size_check），不沿用历史估算。
 */

#ifndef __OBJ_DESC_H
#define __OBJ_DESC_H

#include <stdint.h>
#include "obj_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 对象分层（Q-C2 / B0 待办落地）
 * @note  让"这个对象属于三分类中的哪一类"从注释变成代码可查。
 * ============================================================ */
#define CTRL_OBJ_LAYER_CONTROLLED    1U   /* ① 被控对象   */
#define CTRL_OBJ_LAYER_SYSTEM        2U   /* ② 控制系统对象 */
#define CTRL_OBJ_LAYER_VEHICLE       3U   /* ③ 整车状态对象 */
#define CTRL_OBJ_LAYER_ORCHESTRATOR  4U   /* 编排层（非三分类） */

/* ============================================================
 * 对象描述（全部位于 .rodata，不占 RAM）
 * @note  Q-C8 定稿的 event_mask 语义（**必须遵守，否则会漏路由**）：
 *        - event_mask == 0：接收 [event_base, event_base + 0x1000) 内**全部**事件
 *        - event_mask != 0：只表示区间内 offset 0~31 的位掩码；
 *          offset >= 32 的事件**不会被命中**。要覆盖它们只能置 0 全收，
 *          或另设 event_base 分段。禁止把 event_mask 当成 4096 位图。
 *
 *        Q-C1 定稿：depends_on 保持现有 const uint8_t* 语义（0 结尾），
 *        不改成 const ctrl_obj_type_t*，以免牵动既有代码。
 * ============================================================ */
typedef struct ctrl_obj_desc {
    uint32_t         type;          /* +0  对象类型（对应 ctrl_obj_type_t）*/
    const char      *name;          /* +4  对象名（指向常量串）*/

    uint32_t         event_base;    /* +8  管辖的事件区间起始 */
    uint32_t         event_mask;    /* +12 见上方 Q-C8 语义说明 */

    uint8_t          priority;      /* +16 仲裁权重（REQ-014 用）*/
    uint16_t         required_caps; /* +18 需要哪些能力位（配 sys_caps）*/

    const uint8_t   *depends_on;    /* +20 依赖对象类型列表（0 结尾，NULL 无依赖）*/
    const void      *sm_cfg;        /* +24 StateMachineConfig_t* */
    const void      *obj_cfg;       /* +28 对象专属配置（限位等）*/
    const obj_ops_t *ops;           /* +32 业务虚表 */

    const void      *resources;     /* +36 Q-C1：资源绑定（暂 NULL，字段先行）*/
    uint8_t          layer;         /* +40 Q-C2：①/②/③/编排分层 */
                                    /* +41~43 对齐填充 */
} ctrl_obj_desc_t;

/* 编译期实测（R-06）：大小不符直接编译失败，不靠估算当结论 */
typedef char ctrl_obj_desc_size_check[
    (sizeof(ctrl_obj_desc_t) == 44U) ? 1 : -1
];

/* ============================================================
 * 描述表接口
 * ============================================================ */
extern const ctrl_obj_desc_t g_ctrl_obj_descs[];
extern const uint8_t         g_ctrl_obj_desc_count;

/**
 * @brief 按类型查描述
 * @return 描述指针，未找到返回 NULL
 */
const ctrl_obj_desc_t *obj_desc_find_by_type(uint32_t type);

/**
 * @brief 按事件号查其归属对象描述（替代 app_main 的 if-else 区间路由）
 * @return 描述指针，无归属返回 NULL
 * @note  本批已实现，但 app_main 尚未改为调用它（LEG-3）
 */
const ctrl_obj_desc_t *obj_desc_find_by_event(uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __OBJ_DESC_H */