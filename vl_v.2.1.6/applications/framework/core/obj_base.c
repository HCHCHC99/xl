/**
 * @file    obj_base.c
 * @brief   控制对象基类实现 —— 完全自建，零 RTOS 依赖
 * @note    **本文件严禁 include 任何 RTOS 头文件**，只允许 include：
 *          - obj_base.h（自身）
 *          - obj_desc.h（描述表查询，纯 stdint，零 RTT；2026-11-15 C1 新增）
 *          - framework/state_engine/state_engine.h（纯 stdint，零 RTT）
 *          - framework/port/osal.h（只声明，零 RTT）
 *          - <string.h>（libc，非 RTOS）
 *
 *          验证：编译本文件时定义 OBJ_BASE_NO_RTOS，应 0 error。
 */

#include "obj_base.h"
#include "obj_desc.h"       /* C1：按 type 绑定静态描述（本头文件零 RTOS） */
#include "framework/state_engine/state_engine.h"
#include "framework/port/osal.h"

#include <string.h>

/* ============================================================
 * 内部断言 —— 不使用 sm_assert（其 RT 分支依赖 rtthread.h）
 * 走 osal_assert_fail，生产版可整体空实现
 * ============================================================ */
#ifndef OBJ_ASSERT
    #define OBJ_ASSERT(expr)                                            \
        do {                                                            \
            if (!(expr)) {                                              \
                osal_assert_fail(#expr, __FILE__, __LINE__);            \
            }                                                           \
        } while (0)
#endif

/* ============================================================
 * 初始化
 * ============================================================ */
obj_err_t obj_base_init(obj_base_t *obj,
                        const char *name,
                        uint32_t type,
                        const void *config,
                        void *context,
                        const obj_runtime_cfg_t *runtime,
                        const obj_ops_t *ops)
{
    OBJ_ASSERT(obj != NULL);
    OBJ_ASSERT(name != NULL);

    if (obj == NULL || name == NULL) {
        return OBJ_EINVAL;
    }

    /* 1. 身份标识 */
    obj->name = name;
    obj->type = type;

    /* 2. 配置与上下文 */
    obj->config  = config;
    obj->context = context;

    /* 3. 运行时配置（默认单线程 —— 与原实现语义一致） */
    if (runtime != NULL) {
        obj->runtime_cfg = *runtime;
    } else {
        memset(&obj->runtime_cfg, 0, sizeof(obj_runtime_cfg_t));
        obj->runtime_cfg.use_thread = 0;
    }

    /* 4. 初始模式 */
    obj->current_mode = 0;

    /* 5. 虚表 */
    obj->ops = ops;

    /* 5b. 静态描述（2026-11-15 C1 / Q-C1）：运行时可查自己的描述表项。
     *     表在 .rodata 编译期就绪，此处只读查询，不产生第二份真值；
     *     未填表项的对象（mode / vehicle 等）拿到 NULL，属预期。 */
    obj->desc = obj_desc_find_by_type(type);

    /* 6. 状态机：由调用方在 init 前绑定 obj->sm（保持原行为） */
    /*    若已绑定则初始化；未绑定则跳过（允许无状态机的对象） */
    if (obj->sm != NULL && config != NULL) {
        StateMachine_Init((StateMachine_t *)obj->sm,
                          (const StateMachineConfig_t *)config,
                          context);
    }

    /* 7. 执行器：**不再在此处硬编码选择**（Q-2 行为变更）
     *    改由 ops->init() 自行决定。基类不感知"执行器"概念。 */
    obj->executor      = NULL;
    obj->executor_data = NULL;

    obj->is_initialized = 1;

    osal_log("[ObjBase] %s initialized (type=%u, thread=%u)\n",
             name, (unsigned)type, (unsigned)obj->runtime_cfg.use_thread);

    /* 8. 交给业务对象的 ops->init（执行器绑定、私有资源在此完成） */
    if (obj->ops != NULL && obj->ops->init != NULL) {
        obj_err_t err = obj->ops->init(obj);
        if (err != OBJ_OK) {
            osal_log("[ObjBase] %s ops->init failed (%ld)\n", name, (long)err);
            obj->is_initialized = 0;
            return err;
        }
    }

    return OBJ_OK;
}

/* ============================================================
 * 事件处理
 * ============================================================ */
void obj_base_process(obj_base_t *obj, uint32_t event)
{
    StateMachine_t *sm;

    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return;
    }

    if (obj->sm == NULL) {
        osal_log("[ObjBase] %s has no state machine, event %lu dropped\n",
                 obj->name ? obj->name : "?", (unsigned long)event);
        return;
    }

    if (!obj->is_initialized) {
        osal_log("[ObjBase] %s not initialized, event %lu dropped\n",
                 obj->name ? obj->name : "?", (unsigned long)event);
        return;
    }

    sm = (StateMachine_t *)obj->sm;
    StateMachine_SendEvent(sm, (Event_t)event);
}

/* ============================================================
 * 重置
 * ============================================================ */
void obj_base_reset(obj_base_t *obj)
{
    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return;
    }

    if (obj->sm != NULL) {
        StateMachine_Reset((StateMachine_t *)obj->sm);
    }
}

/* ============================================================
 * 查询接口
 * ============================================================ */
uint32_t obj_base_get_state(const obj_base_t *obj)
{
    OBJ_ASSERT(obj != NULL);

    if (obj == NULL || obj->sm == NULL) {
        return 0;
    }

    return (uint32_t)StateMachine_GetState((StateMachine_t *)obj->sm);
}

void *obj_base_get_context(const obj_base_t *obj)
{
    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return NULL;
    }

    return obj->context;
}

/* ============================================================
 * 运行模式
 * ============================================================ */
void obj_base_set_mode(obj_base_t *obj, uint8_t mode)
{
    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return;
    }

    obj->current_mode = mode;
}

uint8_t obj_base_get_mode(const obj_base_t *obj)
{
    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return 0xFF;
    }

    return obj->current_mode;
}

/* ============================================================
 * 事件投递（经由执行器）
 * @note  executor 是不透明句柄。为使 obj_base.c 不依赖 ctrl_executor.h
 *        （后者 include 了 rtthread.h 与 app_event.h），此处采用
 *        **最小接口约定**：executor 结构体前三个字段必须是
 *        name / init / emit，emit 位于第 3 个指针槽。
 *
 *        【2026-11-23 修复】原实现只声明 {name, emit}，把第 2 槽（实际是
 *        init）当成 emit 调用 ⇒ mode_control_set_mode() 等经
 *        control_object_emit() 投递的事件**全部被吞**，表现为上电默认
 *        模式停在 INIT、t_wmode/mode_set 无效。
 *        现按 ctrl_executor_t 真实布局补齐 init 槽位；
 *        ctrl_executor.c 增加了 offsetof 断言，防止再次错位。
 * ============================================================ */
typedef obj_err_t (*obj_executor_emit_fn)(void *self, obj_base_t *obj, uint32_t event);

typedef struct {
    const char              *name;
    void                    *init;      /* 占位：与 ctrl_executor_t.init 对齐 */
    obj_executor_emit_fn     emit;      /* 第 3 个指针槽 = ctrl_executor_t.emit */
} obj_executor_iface_t;

/* 本文件内部自校验：name + init + emit 三个指针槽，无填充 */
typedef char obj_executor_iface_size_check[
    (sizeof(obj_executor_iface_t) == (sizeof(void *) * 3)) ? 1 : -1
];

obj_err_t obj_base_emit(obj_base_t *obj, uint32_t event)
{
    const obj_executor_iface_t *iface;

    OBJ_ASSERT(obj != NULL);

    if (obj == NULL) {
        return OBJ_EINVAL;
    }

    if (obj->executor == NULL) {
        /* 无执行器：退化为同步处理，保证事件不丢（原实现直接报错） */
        obj_base_process(obj, event);
        return OBJ_OK;
    }

    iface = (const obj_executor_iface_t *)obj->executor;
    if (iface->emit == NULL) {
        return OBJ_ENOTSUP;
    }

    return iface->emit(obj->executor, obj, event);
}
