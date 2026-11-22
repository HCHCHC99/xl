/**
 * @file    can_tx.h
 * @brief   CAN 周期发送标准化接口
 * @note    时基 = 周期软定时器；发送 = 独立线程；组包 = 各模块注册 pack 回调
 */
#ifndef __CAN_TX_H
#define __CAN_TX_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TX_DLC_MAX      8U      /* 标准 CAN 数据域 */
#define CAN_TX_SLOT_MAX     8U      /* 最大注册帧数 */

/* -------- 统一的组包容器 -------- */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[CAN_TX_DLC_MAX];
} can_tx_frame_t;

/**
 * @brief 标准化组包回调（业务模块唯一需要实现的东西）
 * @param [out] frame 输出容器，data 已清零，只需填内容
 */
typedef void (*can_pack_fn_t)(can_tx_frame_t *frame);

/* -------- 发送触发策略 -------- */
typedef enum {
    CAN_TX_TRIG_PERIODIC = 0,   /* 仅周期发送 */
    CAN_TX_TRIG_ONCHANGE = 1,   /* 变化即发 + 周期保底 */
} can_tx_trig_t;

/* -------- 注册项 -------- */
typedef struct {
    const char   *name;
    uint32_t      id;
    uint8_t       dlc;
    uint16_t      period_ms;    /* 0 = 不参与周期调度 */
    uint16_t      phase_ms;     /* 首帧延迟(ms)：仅决定首次发送时刻，
                                 * 之后再按 period_ms 稳定发送；
                                 * 严禁叠加到每周期，否则周期变为 period+phase */
    can_tx_trig_t trig;
    can_pack_fn_t pack;
} can_tx_entry_t;

/* -------- API -------- */
int  can_tx_register(const can_tx_entry_t *entry);  /* [单线程] 初始化期调用 */
void can_tx_force(uint32_t id);                     /* [线程安全] 变更即发 */
int  can_tx_init(void);                             /* [单线程] 创建时基与线程 */
void can_tx_poll(uint32_t elapsed_ms);              /* [单线程] 调度核心 */
void can_tx_dump(void);                             /* [线程安全] 调试 */

#ifdef __cplusplus
}
#endif
#endif /* __CAN_TX_H */
