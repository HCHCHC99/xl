/**
 * @file    obj_base.h
 * @brief   控制对象基类 —— 完全自建，零 RTOS 依赖
 * @note    **本文件严禁 include 任何 RTOS 头文件**（这是方案的硬约束）。
 *          验证：定义 OBJ_BASE_NO_RTOS 后单独编译本头文件所在模块，应 0 error。
 *
 *          设计要点（PLAN-OBJ-BASE-V1.0）：
 *          1. 36 B 原有字段全部保留 → 业务侧结构体布局不变
 *          2. 新增 const obj_ops_t *ops（4 B）→ 业务层虚表
 *          3. sm / config 用 void* → 断开对状态机引擎的类型依赖
 *          4. name 用 const char*（非 char[16]）→ 省 12 B/对象
 *
 *          2026-11-15 C1 批（Q-C1 按建议代裁定）：
 *          5. 新增 const struct ctrl_obj_desc *desc（4 B）→ 运行时可查自己的描述，
 *             基类 40 B → 44 B。枚举 ctrl_obj_type_t 由 control_object.h 上移到本文件，
 *             使 framework/core/obj_desc.c 填表时仍能保持零 RTOS 依赖。
 */

#ifndef __OBJ_BASE_H
#define __OBJ_BASE_H

#include <stdint.h>
#include <stddef.h>     /* offsetof */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 错误码（替代 rt_err_t，语义与 RT_EOK 对齐：0 = 成功）
 * ============================================================ */
typedef int32_t obj_err_t;

#define OBJ_OK          (0)
#define OBJ_ERROR       (-1)
#define OBJ_EINVAL      (-2)
#define OBJ_ENOMEM      (-3)
#define OBJ_EFULL       (-4)
#define OBJ_ENOTSUP     (-5)

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct obj_base obj_base_t;
typedef struct obj_ops  obj_ops_t;

/* 描述表结构前置声明（避免 obj_base.h 反向依赖 obj_desc.h） */
struct ctrl_obj_desc;

/* ============================================================
 * 控制对象类型枚举
 * @note  2026-11-15 C1 批由 control_object.h 上移到本文件：
 *        obj_desc.c 填描述表需要按 type 引用，若去 include control_object.h
 *        会连带引入 <rtthread.h>，破坏 framework/core 的零 RTOS 约束。
 *        枚举值逐项保持原样，业务侧 include control_object.h 后照常可见。
 * ============================================================ */
typedef enum {
    CTRL_OBJ_TYPE_MODE    = 0,      /* 模式控制 */
    CTRL_OBJ_TYPE_SWING   = 1,      /* 回转控制 */
    CTRL_OBJ_TYPE_LIFT    = 2,      /* 举升控制 */
    CTRL_OBJ_TYPE_SEQ     = 3,      /* 序列控制 */
    CTRL_OBJ_TYPE_CALIB   = 4,      /* 校准控制 */
    CTRL_OBJ_TYPE_CLUTCH  = 5,      /* 离合控制（选配） */
    CTRL_OBJ_TYPE_VEHICLE = 6,      /* 整车状态 */
    CTRL_OBJ_TYPE_MAX
} ctrl_obj_type_t;

/* ============================================================
 * 运行时配置（语义与原 ctrl_obj_runtime_cfg_t 完全一致）
 * ============================================================ */
typedef struct {
    uint8_t  use_thread;         /* 1=独立线程，0=共享主线程 */
    uint8_t  thread_prio;        /* 线程优先级（仅多线程有效） */
    uint16_t stack_size;         /* 线程栈大小（仅多线程有效） */
    uint16_t mq_size;            /* 消息队列容量（仅多线程有效） */
} obj_runtime_cfg_t;

/* ============================================================
 * 业务对象虚表
 * @note  3 个方法（Q-1）。若后续要做业务策略虚表化（G-04），
 *        在此追加指针即可，静态实例在 .rodata，不占 RAM。
 * ============================================================ */
struct obj_ops {
    /**
     * @brief 对象初始化（在基类 init 末尾被调用）
     * @note  执行器绑定、私有资源分配等"对象自己的事"放在这里，
     *        基类不感知"执行器"概念（对应 Q-2 的行为变更）
     */
    obj_err_t (*init)(obj_base_t *self);

    /**
     * @brief 对象反初始化（释放私有资源）
     */
    obj_err_t (*deinit)(obj_base_t *self);

    /**
     * @brief 事件处理入口（事件路由的目标函数）
     */
    obj_err_t (*handle)(obj_base_t *self, uint32_t event);
};

/* ============================================================
 * 控制对象基类（C1 批后实测 44 B；C1 前为 40 B）
 * ============================================================ */
struct obj_base {
    /* ----- 身份标识 ----- */
    const char      *name;          /* +0   */
    uint32_t         type;          /* +4   */

    /* ----- 状态机（void* 断开类型依赖） ----- */
    void            *sm;            /* +8   */

    /* ----- 业务上下文 ----- */
    void            *context;       /* +12  */
    const void      *config;        /* +16  */

    /* ----- 执行器（void* 不透明句柄） ----- */
    void            *executor;      /* +20  */
    void            *executor_data; /* +24  */

    /* ----- 运行时配置 ----- */
    obj_runtime_cfg_t runtime_cfg;  /* +28 (6 B) */

    /* ----- 运行模式 ----- */
    uint8_t          current_mode;  /* +34  */

    /* ----- 状态 ----- */
    uint8_t          is_initialized;/* +35  */

    /* ----- 虚表（新增 4 B） ----- */
    const obj_ops_t *ops;           /* +36  */

    /* ----- 静态描述（Q-C1：运行时可查"我是什么/管哪些事件"） ----- */
    const struct ctrl_obj_desc *desc;  /* +40，init 时按 type 自动绑定 */
};

/* 编译期实测（R-06）：C1 批加 desc 后为 44 B，不符直接编译失败 */
typedef char obj_base_size_check[(sizeof(obj_base_t) == 44U) ? 1 : -1];

/* ============================================================
 * 零依赖 container_of（3 行自写，不引 rt_container_of）
 * ============================================================ */
#define obj_container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

/* ============================================================
 * 基类 API —— 与原 control_object_* 一一对应
 * ============================================================ */

/**
 * @brief 初始化控制对象
 * @param obj       基类指针
 * @param name      对象名（指向常量字符串，不做拷贝）
 * @param type      对象类型
 * @param config    状态机配置（可为 NULL，由调用方自行保证 sm 已绑定）
 * @param context   业务上下文
 * @param runtime   运行时配置（NULL = 默认单线程）
 * @param ops       业务虚表（可为 NULL，则无 ops 回调）
 * @return OBJ_OK 成功
 */
obj_err_t obj_base_init(obj_base_t *obj,
                        const char *name,
                        uint32_t type,
                        const void *config,
                        void *context,
                        const obj_runtime_cfg_t *runtime,
                        const obj_ops_t *ops);

/**
 * @brief 同步处理事件（转发到状态机）
 */
void obj_base_process(obj_base_t *obj, uint32_t event);

/**
 * @brief 重置到初始状态
 */
void obj_base_reset(obj_base_t *obj);

/**
 * @brief 获取当前状态
 */
uint32_t obj_base_get_state(const obj_base_t *obj);

/**
 * @brief 获取业务上下文
 */
void *obj_base_get_context(const obj_base_t *obj);

/**
 * @brief 设置运行模式
 */
void obj_base_set_mode(obj_base_t *obj, uint8_t mode);

/**
 * @brief 获取运行模式
 */
uint8_t obj_base_get_mode(const obj_base_t *obj);

/**
 * @brief 通过执行器投递事件
 * @note  executor 与 executor_data 均为不透明句柄，具体的投递动作由
 *        executor 侧的 emit 回调完成（见 ctrl_executor.h）
 */
obj_err_t obj_base_emit(obj_base_t *obj, uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __OBJ_BASE_H */