/**
 * @file    sys_param.h
 * @brief   系统参数数据对象（业务模块）
 */

#ifndef __SYS_PARAM_H
#define __SYS_PARAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 齿数类型
 * ============================================================ */
typedef enum {
    TEETH_UNSET = 0,
    TEETH_11    = 1,
    TEETH_13    = 2,
    TEETH_14    = 3
} teeth_type_t;

/* ============================================================
 * 清除机型设置后的行为（Q4 可配置项）
 * ============================================================ */
typedef enum {
    TEETH_CLEAR_KEEP = 0,   /* 仅清 EEP，控制器保持当前齿型继续运行 */
    TEETH_CLEAR_STOP = 1    /* 清 EEP 并停止输出，等待重新配置 */
} teeth_clear_action_t;

/* ============================================================
 * 系统参数结构体
 * ============================================================
 * 注意：本结构直接落盘，字段顺序与 sizeof 不得随意变更；
 *       新增配置优先复用 reserved 字段，避免破坏已存数据布局。
 * ============================================================ */
typedef struct {
    /* ============================================================
     * 【2026-11-03 新增】记录识别魔数，必须在首部
     * ------------------------------------------------------------
     * 用途一（关键）：保证默认值非零
     *   本结构若干字段默认值为 0（如 enc_zero_raw[]、is_calibrated），
     *   若整体默认值全 0，"空分区写默认值"时
     *   memset(last_data_buf, 0) 后与 data_ptr 相同，
     *   会被 storage_save 的值去重 (storage.c:185) 静默跳过，
     *   默认值永远写不进 Flash。加非零字段后清零必然不等。
     *
     * 用途二：记录完整性校验
     *   magic 不符 = 该记录不是本版本写的（旧格式 / 位翻转），
     *   load 后校验并回默认，避免"CRC 通过但语义错误"的脏数据。
     *
     * ⚠️ 放在首部而非末尾的副作用：
     *   旧 Flash 数据的首 4 字节是 lift_zero_voltage_mv(2203=0x089B)
     *   + 填充，必然 != 0x53595052，校验会失败并回默认。
     *   这是期望行为 —— 旧数据本就不可信。
     *   当前设备 Flash 从未成功持久化过，无实际数据损失。
     * ============================================================ */
    uint32_t rec_magic;

    /* ----- 举升相关 ----- */
    uint16_t lift_zero_voltage_mv;     /* 举升传感器零位电压 (mV) */
    int32_t  lift_encoder_zero_raw;    /* 保留字段 */

    /* ----- 回转相关 ----- */
    int32_t  encoder_zero_raw;          /* 旧字段，保留兼容；新逻辑用下面的数组 */

    /* 【2026-11-02 新增】三颗磁编码器零点偏移（方案 A：三颗独立保存）
     *
     * 为什么存三颗而不是存解算后的零位：
     *   游标解算用的是三颗的"相对关系"。若只存一个解算零位，
     *   某颗编码器维修更换后就无法知道"哪颗偏了多少"，
     *   也无法只重校某一颗。存三颗原始值可单独查、单独校。
     *
     * 取值范围：12bit 角度 0~4095（当前按 12bit 使用；
     *   若将来确认芯片是 16bit 输出，改用 0~65535，字段需扩为 uint16 仍够用）
     *
     * 应用方式（abs_angle.c 读三颗之后、进游标循环之前）：
     *   raw_n = (raw_n - enc_zero_raw[n] + ENC_FULL) % ENC_FULL;
     * 这样游标算法本身完全不用改。 */
    uint16_t enc_zero_raw[3];           /* [0]=Z0(齿41) [1]=Z1(齿42) [2]=Z2(齿43) */

    /* 三颗零偏是否已有效标定（与 is_calibrated 语义不同：
     * is_calibrated 是整体校准标志，本字段专指"零点是否采过"） */
    uint8_t  enc_zero_valid;

    /* ----- 通用 ----- */
    uint8_t  teeth_type;
    uint8_t  is_calibrated;
    uint8_t  teeth_clear_action;        /* teeth_clear_action_t（原 reserved[0]） */
    uint8_t  reserved;                  /* 原 reserved[1] */
} sys_param_t;

/* 记录识别魔数（"SYPR" = SYsParam Record）
 * 与分区 magic 0x53595350 ("SYSP") 区分，避免两者混淆。 */
#define SYS_PARAM_REC_MAGIC     0x53595052U

/* ============================================================
 * 外部引用
 * ============================================================ */
extern sys_param_t g_sys_params;

/* ============================================================
 * API
 * ============================================================ */
int sys_param_init(void);
int sys_param_save(void);
int sys_param_load(void);
sys_param_t* sys_param_get(void);
void sys_param_get_stats(uint32_t *write_count, uint32_t *skip_count);

/* 便捷接口
 * 说明：以下 setter 内部均做「值变化」判断，值未变时直接返回，
 *       不会触发 Flash 擦写（storage 层另有整块 memcmp 兜底）。 */
void sys_param_set_teeth(uint8_t type);
void sys_param_set_lift_zero_voltage(uint16_t mv);
void sys_param_set_calibrated(uint8_t calibrated);
void sys_param_set_teeth_clear_action(uint8_t action);

/**
 * @brief 清除零点与校准标志（一次 Flash 写，V1.3 / C-08）
 * @note  切换机械 profile 后必须调用：旧零点在新齿数模型下无意义。
 *        同时清 encoder_zero_raw / lift_zero_voltage_mv / is_calibrated。
 */
void sys_param_clear_calibration(void);

/* ============================================================
 * 【2026-11-02 新增】三颗编码器零点校准（方案 A）
 * ============================================================ */

/**
 * @brief  保存三颗编码器的零点偏移
 * @param  z0/z1/z2  各颗当前的 12bit 角度原始值（0~4095）
 * @retval 0 成功；<0 失败
 * @note   只写 flash，不读传感器。调用方负责先采到当前角度。
 *         内部会同时置 enc_zero_valid=1 与 is_calibrated=1。
 */
int sys_param_set_enc_zero(uint16_t z0, uint16_t z1, uint16_t z2);

/**
 * @brief  读取三颗编码器零点偏移
 * @param  out  输出数组，长度 >= 3；可为 NULL（只查有效性）
 * @retval 1 有效（enc_zero_valid=1）；0 未校准
 */
uint8_t sys_param_get_enc_zero(uint16_t out[3]);

/**
 * @brief  清除三颗编码器零点（复位校准标记，允许重新校准）
 * @note   会把 enc_zero_raw[] 清零、enc_zero_valid=0、is_calibrated=0，
 *         并落盘。清除后系统视为"未校准"，可用性由上层决定。
 */
void sys_param_clear_enc_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_PARAM_H */
