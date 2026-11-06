/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pid_ctrl.h - PID 控制器公共头文件（裸机无堆版本）
 *
 * 设计目标：
 *   1. 裸机零依赖、零锁、零 malloc，全部静态分配，可直接编译运行；
 *   2. 迁移 RT-Thread 只需编译时加 -DPID_USE_RTTHREAD，业务代码无需改动；
 *   3. 通过 ops 接口对外暴露能力，便于后续扩展不同 PID 变种。
 *
 * 功能特性：
 *   - 位置式 / 增量式 双模式；
 *   - 微分先行（对反馈微分，抑制给定阶跃带来的微分冲击）；
 *   - 不完全微分（微分项一阶低通滤波，抑制高频噪声）；
 *   - 反计算抗积分饱和（输出限幅时回退积分，避免积分堆死）；
 *   - 速度前馈 + 加速度前馈（提升跟踪性能）；
 *   - 未限幅输出查询（供级联控制诊断"下游饱和"）。
 *
 * 内存模型：
 *   不使用任何动态内存。调用者需提供 struct pid_dev 与 struct pid_priv
 *   实例（通常为全局/静态变量），通过 pid_dev_init() 初始化。
 *
 * 量纲提示（重要）：
 *   位置式积分项为 ki * integral，其中 integral += err * dt。
 *   因此 ki 的实际作用等效于 "每单位时间累积 err 的增益"，
 *   与"每拍直接累加 ki*err"的写法相差 1/dt 因子。
 *   20kHz 下 dt = 5e-5，若按后者整定，此处的 ki 需放大 20000 倍才等效。
 *   整定时以实际阶跃响应为准，不要照搬其他 PID 的 ki 数值。
 */

#ifndef __PID_CTRL_H
#define __PID_CTRL_H

#include <stdbool.h>
#include <stddef.h>

/*
 * 环境适配层：裸机(默认) / RT-Thread(-DPID_USE_RTTHREAD)
 *
 * 锁用宏封装，业务代码只调宏，不直接依赖任何 OS API。
 * 注意：本版本不提供内存分配宏，全部静态分配。
 */
#ifdef PID_USE_RTTHREAD
/* RT-Thread 环境：使用互斥量做并发保护 */
#include <rtthread.h>

typedef struct rt_mutex pid_lock_t;

#define PID_LOCK_INIT(l)	rt_mutex_init((l), "pid", RT_IPC_FLAG_PRIO)
#define PID_LOCK_DESTROY(l)	rt_mutex_detach(l)
#define PID_LOCK(l)		rt_mutex_take((l), RT_WAITING_FOREVER)
#define PID_UNLOCK(l)		rt_mutex_release(l)
#else
/*
 * 裸机环境（默认）：空锁结构体 + 空宏，编译器会完全优化掉，
 * 不产生任何运行时开销。
 */
typedef struct { int dummy; } pid_lock_t;

#define PID_LOCK_INIT(l)	do { (void)(l); } while (0)
#define PID_LOCK_DESTROY(l)	do { (void)(l); } while (0)
#define PID_LOCK(l)		do { (void)(l); } while (0)
#define PID_UNLOCK(l)		do { (void)(l); } while (0)
#endif /* PID_USE_RTTHREAD */

/**
 * enum pid_mode - PID 控制模式
 * @PID_MODE_POSITION:	位置式，输出为控制量绝对值，适合位置环、阀门、伺服
 * @PID_MODE_INCREMENT:	增量式，输出为控制量增量，适合速度环、步进、执行器自带积分
 */
enum pid_mode {
	PID_MODE_POSITION,
	PID_MODE_INCREMENT,
};

/**
 * struct pid_priv - PID 私有参数与运行状态
 * @lock:	并发保护锁（裸机为空，RT-Thread 为互斥量）
 * @mode:	控制模式：位置式 / 增量式
 * @kp:		比例增益
 * @ki:		积分增益（注意量纲：见文件头"量纲提示"）
 * @kd:		微分增益
 * @k_ffv:	速度前馈增益，乘以目标一阶微分 ref_dot
 * @k_ffa:	加速度前馈增益，乘以目标二阶微分 ref_ddot
 * @en_ffv:	速度前馈使能
 * @en_ffa:	加速度前馈使能
 * @diff_alpha:	不完全微分低通系数 alpha∈[0,1]；1=纯微分无滤波，越小滤波越强
 * @out_max:	PID 输出上限
 * @out_min:	PID 输出下限
 * @back_gain:	反计算回退系数，一般 0~1，越接近 1 回退越强
 * @setpoint:	目标给定 ref
 * @feedback:	当前反馈 fb
 * @err:	当前误差 e(k) = ref - fb
 * @err_last:	上一拍误差 e(k-1)
 * @err_prev:	上上拍误差 e(k-2)
 * @integral:	积分累加项（已含 dt 因子）
 * @diff_raw:	原始微分值（微分先行：对反馈微分）
 * @diff_filtered: 不完全微分滤波后的微分值
 * @output:	上一拍最终输出（含前馈、限幅），增量式累加基准
 * @raw_output:	未限幅的 PID 本体输出（不含前馈），饱和诊断用
 * @feedback_last: 上一拍反馈值，用于微分先行计算
 *
 * 该结构体由调用者静态分配，内部包含锁，不可随意拷贝。
 */
struct pid_priv {
	pid_lock_t	lock;
	enum pid_mode	mode;

	float		kp;
	float		ki;
	float		kd;

	float		k_ffv;
	float		k_ffa;
	bool		en_ffv;
	bool		en_ffa;

	float		diff_alpha;

	float		out_max;
	float		out_min;
	float		back_gain;

	float		setpoint;
	float		feedback;
	float		err;
	float		err_last;
	float		err_prev;
	float		integral;
	float		diff_raw;
	float		diff_filtered;
	float		output;
	float		raw_output;
	float		feedback_last;
};

/**
 * struct pid_ops - PID 操作函数集
 * @set_param:	设置 PID 基础三参数 kp/ki/kd，返回 0 成功
 * @set_feedforward: 设置前馈增益与前馈使能，返回 0 成功
 * @set_diff_filter: 设置不完全微分低通系数 alpha∈[0,1]，参数非法返回 -1
 * @set_limit:	设置输出限幅与反计算回退系数，参数非法返回 -1
 * @set_ref:	单独更新目标给定（可选，calc 里也会传入 ref）
 * @calc:	PID 核心计算，返回经过限幅后的最终控制输出
 * @get_raw_output: 返回最近一次 calc 的未限幅本体输出（不含前馈）
 * @reset:	复位内部状态：误差、积分、微分缓存全部清零
 *
 * Linux 驱动风格 ops，方便后续扩展不同 PID 变种而不改动业务代码。
 * 所有函数第一个参数均为私有结构体指针。
 */
struct pid_ops {
	int	(*set_param)(struct pid_priv *priv, float kp, float ki, float kd);
	int	(*set_feedforward)(struct pid_priv *priv, float k_ffv, float k_ffa,
				   bool en_ffv, bool en_ffa);
	int	(*set_diff_filter)(struct pid_priv *priv, float alpha);
	int	(*set_limit)(struct pid_priv *priv, float out_min, float out_max,
			     float back_gain);
	int	(*set_ref)(struct pid_priv *priv, float ref);

	/*
	 * calc - PID 核心计算
	 * @priv:	PID 私有结构体
	 * @fb:		当前反馈值
	 * @ref:	目标值
	 * @ref_dot:	目标一阶微分（速度），供速度前馈
	 * @ref_ddot:	目标二阶微分（加速度），供加速度前馈
	 * @dt:		控制周期，单位秒（必须 > 0，否则返回上次输出）
	 *
	 * 返回限幅后的最终控制输出。
	 */
	float	(*calc)(struct pid_priv *priv, float fb, float ref,
			float ref_dot, float ref_ddot, float dt);

	/*
	 * get_raw_output - 返回最近一次 calc 的未限幅输出
	 * @priv:	PID 私有结构体
	 *
	 * 用于判断"PID 想给多少"与"实际限幅后给了多少"的差，
	 * 诊断是否因输出限幅导致跟踪不上。未调用过 calc 时返回 0。
	 */
	float	(*get_raw_output)(struct pid_priv *priv);

	void	(*reset)(struct pid_priv *priv);
};

/**
 * struct pid_dev - 对外设备句柄
 * @priv:	私有参数与状态（指向调用者提供的实例）
 * @ops:	操作接口
 */
struct pid_dev {
	struct pid_priv		*priv;
	const struct pid_ops	*ops;
};

/*
 * 静态实例接口（本版本唯一创建方式）
 *
 * 调用者自行提供 dev 与 priv 内存（全局/静态变量），init 成功后可直接使用。
 * 不涉及任何动态内存分配。
 * 返回 0 成功，-1 参数非法（含 dev/priv 为 NULL、mode 越界）。
 */
int pid_dev_init(struct pid_dev *dev, struct pid_priv *priv, enum pid_mode mode);
void pid_dev_deinit(struct pid_dev *dev);

#ifdef PID_USE_RTTHREAD
/*
 * 可选动态接口（仅 RT-Thread 环境编译）
 *
 * 若 RT-Thread 应用需要动态创建 PID 实例，可启用这两个接口。
 * 裸机环境不编译，避免引入堆依赖。
 */
struct pid_dev *pid_dev_create(enum pid_mode mode);
void pid_dev_destroy(struct pid_dev *dev);
#endif /* PID_USE_RTTHREAD */

#endif /* __PID_CTRL_H */
