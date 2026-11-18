/**
 * @file    ctrl_manager.h
 * @brief   控制对象管理器 —— 统一注册、查找、广播、调试
 */

#ifndef __CTRL_MANAGER_H
#define __CTRL_MANAGER_H

#include <stdint.h>
#include "../core/control_object.h"

/* 过渡：本批管理器的 API 仍返回 rt_err_t（LEG 项）
 * 定义 OBJ_BASE_NO_RTOS 时改用 ctrl_obj_err_t，用于零 RTT 自检 */
#ifndef OBJ_BASE_NO_RTOS
    #include <rtthread.h>
    typedef rt_err_t ctrl_mgr_err_t;
#else
    typedef ctrl_obj_err_t ctrl_mgr_err_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 管理器最大对象数
 * ============================================================ */
#ifndef CTRL_MGR_MAX_OBJECTS
#define CTRL_MGR_MAX_OBJECTS    16
#endif

/* ============================================================
 * 框架错误码
 * ============================================================ */
#define CTRL_OK             (0)
#define CTRL_ERR            (1)     /* 通用错误 */
#define CTRL_ERR_FULL       (2)     /* 对象表满 */
#define CTRL_ERR_EXIST      (3)     /* 对象已存在 */
#define CTRL_ERR_NOT_FOUND  (4)     /* 对象未找到 */
#define CTRL_ERR_NO_MEM     (5)     /* 内存不足 */
#define CTRL_ERR_INVAL      (6)     /* 参数无效 */

/* ============================================================
 * 管理器结构
 * ============================================================ */
typedef struct {
    control_object_t *objects[CTRL_MGR_MAX_OBJECTS];
    uint8_t           count;
    uint8_t           initialized;
    uint8_t           running;
} ctrl_manager_t;

/* ============================================================
 * 生命周期 API
 * ============================================================ */

/**
 * @brief 初始化管理器
 */
ctrl_mgr_err_t ctrl_manager_init(ctrl_manager_t *mgr);

/**
 * @brief 启动所有控制对象（调用执行器的 init）
 */
ctrl_mgr_err_t ctrl_manager_start(ctrl_manager_t *mgr);

/**
 * @brief 停止所有控制对象
 */
void ctrl_manager_stop(ctrl_manager_t *mgr);

/* ============================================================
 * 对象管理 API
 * ============================================================ */

/**
 * @brief 注册控制对象
 */
ctrl_mgr_err_t ctrl_manager_register(ctrl_manager_t *mgr, control_object_t *obj);

/**
 * @brief 反注册控制对象
 */
ctrl_mgr_err_t ctrl_manager_unregister(ctrl_manager_t *mgr, control_object_t *obj);

/**
 * @brief 按名称查找
 */
control_object_t* ctrl_manager_find_by_name(ctrl_manager_t *mgr, const char *name);

/**
 * @brief 按类型查找
 */
control_object_t* ctrl_manager_find_by_type(ctrl_manager_t *mgr, ctrl_obj_type_t type);

/* ============================================================
 * 事件分发 API
 * ============================================================ */

/**
 * @brief 广播事件到所有对象（通过 run，同步执行）
 */
ctrl_mgr_err_t ctrl_manager_broadcast(ctrl_manager_t *mgr, Event_t  event);

/**
 * @brief 定向投递事件到特定类型的对象（通过 emit，异步）
 */
ctrl_mgr_err_t ctrl_manager_emit_to(ctrl_manager_t *mgr,
                               ctrl_obj_type_t type,
                               Event_t  event);

/* ============================================================
 * 模式管理 API
 * ============================================================ */

/**
 * @brief 设置所有对象的运行模式
 */
void ctrl_manager_set_mode(ctrl_manager_t *mgr, uint8_t mode);

/**
 * @brief 获取指定对象的模式
 */
uint8_t ctrl_manager_get_mode(ctrl_manager_t *mgr, ctrl_obj_type_t type);

/* ============================================================
 * 调试 API
 * ============================================================ */

/**
 * @brief 打印所有对象状态
 */
void ctrl_manager_dump(ctrl_manager_t *mgr);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_MANAGER_H */
