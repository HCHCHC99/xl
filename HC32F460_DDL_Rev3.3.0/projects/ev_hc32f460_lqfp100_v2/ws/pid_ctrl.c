// SPDX-License-Identifier: GPL-2.0
/*
 * pid_ctrl.c - PID 控制器实现（裸机无堆版本）
 *
 * 特性：
 *   - 位置式 / 增量式 双模式；
 *   - 微分先行 + 不完全微分；
 *   - 反计算抗积分饱和；
 *   - 速度前馈 + 加速度前馈；
 *   - dt 非法保护；
 *   - 未限幅输出查询（get_raw_output）；
 *   - 零动态内存，全部静态分配。
 *
 * 相对初版的健壮性修正：
 *   - pid_calc 的 dt 检查移到加锁之前，避免早期返回时锁未释放；
 *   - pid_reset 额外清零 setpoint/feedback；
 *   - pid_dev_init 校验 mode 合法性；
 *   - 各 set_* 接口对 priv==NULL 做保护；
 *   - pid_dev_deinit 后清空句柄指针；
 *   - pid_priv_defaults 不再依赖 <string.h>；
 *   - 新增 raw_output 字段与 get_raw_output，供级联控制饱和诊断。
 */

#include "pid_ctrl.h"

/* 无 libc 依赖的整块清零：逐字段赋值，编译器可优化成高效序列 */
static void pid_priv_clear(struct pid_priv *priv)
{
	priv->mode		= PID_MODE_POSITION;
	priv->kp		= 0.0f;
	priv->ki		= 0.0f;
	priv->kd		= 0.0f;
	priv->k_ffv		= 0.0f;
	priv->k_ffa		= 0.0f;
	priv->en_ffv		= false;
	priv->en_ffa		= false;
	priv->diff_alpha	= 0.3f;
	priv->out_max		= 1.0f;
	priv->out_min		= -1.0f;
	priv->back_gain		= 0.8f;
	priv->setpoint		= 0.0f;
	priv->feedback		= 0.0f;
	priv->err		= 0.0f;
	priv->err_last		= 0.0f;
	priv->err_prev		= 0.0f;
	priv->integral		= 0.0f;
	priv->diff_raw		= 0.0f;
	priv->diff_filtered	= 0.0f;
	priv->output		= 0.0f;
	priv->raw_output	= 0.0f;
	priv->feedback_last	= 0.0f;
}

/* ==========================================================================
 * set 系列：参数配置接口
 *
 * 所有 set 接口都加锁，保证与 calc 并发调用时参数一致性。
 * 全部对 priv==NULL 做保护，避免误用崩溃。
 * ========================================================================== */

/**
 * pid_set_param() - 设置 PID 基础三参数
 * @priv:	PID 私有结构体
 * @kp:		比例增益
 * @ki:		积分增益（量纲见头文件"量纲提示"）
 * @kd:		微分增益
 *
 * 返回 0 成功，-1 priv 为 NULL。
 */
static int pid_set_param(struct pid_priv *priv, float kp, float ki, float kd)
{
	if (!priv)
		return -1;

	PID_LOCK(&priv->lock);
	priv->kp = kp;
	priv->ki = ki;
	priv->kd = kd;
	PID_UNLOCK(&priv->lock);

	return 0;
}

/**
 * pid_set_feedforward() - 设置速度/加速度前馈增益与使能
 * @priv:	PID 私有结构体
 * @k_ffv:	速度前馈增益
 * @k_ffa:	加速度前馈增益
 * @en_ffv:	速度前馈使能
 * @en_ffa:	加速度前馈使能
 *
 * 返回 0 成功，-1 priv 为 NULL。
 */
static int pid_set_feedforward(struct pid_priv *priv, float k_ffv, float k_ffa,
			       bool en_ffv, bool en_ffa)
{
	if (!priv)
		return -1;

	PID_LOCK(&priv->lock);
	priv->k_ffv  = k_ffv;
	priv->k_ffa  = k_ffa;
	priv->en_ffv = en_ffv;
	priv->en_ffa = en_ffa;
	PID_UNLOCK(&priv->lock);

	return 0;
}

/**
 * pid_set_diff_filter() - 设置不完全微分低通滤波系数
 * @priv:	PID 私有结构体
 * @alpha:	滤波系数 [0,1]；1=纯微分无滤波
 *
 * 返回 0 成功，-1 参数非法。
 */
static int pid_set_diff_filter(struct pid_priv *priv, float alpha)
{
	if (!priv)
		return -1;
	if (alpha < 0.0f || alpha > 1.0f)
		return -1;	/* 参数非法，不修改内部状态 */

	PID_LOCK(&priv->lock);
	priv->diff_alpha = alpha;
	PID_UNLOCK(&priv->lock);

	return 0;
}

/**
 * pid_set_limit() - 设置输出限幅与反计算抗饱和回退系数
 * @priv:	PID 私有结构体
 * @out_min:	输出下限
 * @out_max:	输出上限
 * @back_gain:	反计算回退系数，一般 0~1
 *
 * 限幅上下界颠倒/相等或回退系数越界都视为非法，返回 -1。
 */
static int pid_set_limit(struct pid_priv *priv, float out_min, float out_max,
			 float back_gain)
{
	if (!priv)
		return -1;
	if (out_min >= out_max || back_gain < 0.0f || back_gain > 1.0f)
		return -1;

	PID_LOCK(&priv->lock);
	priv->out_min   = out_min;
	priv->out_max   = out_max;
	priv->back_gain = back_gain;
	PID_UNLOCK(&priv->lock);

	return 0;
}

/**
 * pid_set_ref() - 单独设置目标给定
 * @priv:	PID 私有结构体
 * @ref:	目标值
 *
 * 返回 0 成功，-1 priv 为 NULL。
 */
static int pid_set_ref(struct pid_priv *priv, float ref)
{
	if (!priv)
		return -1;

	PID_LOCK(&priv->lock);
	priv->setpoint = ref;
	PID_UNLOCK(&priv->lock);

	return 0;
}

/* ==========================================================================
 * reset：清空 PID 内部全部历史状态
 *
 * 使用场景：上电、启停、切模式、急停之后调用，防止旧积分/微分残留冲击。
 * ========================================================================== */
static void pid_reset(struct pid_priv *priv)
{
	if (!priv)
		return;

	PID_LOCK(&priv->lock);
	priv->setpoint		= 0.0f;
	priv->feedback		= 0.0f;
	priv->err		= 0.0f;
	priv->err_last		= 0.0f;
	priv->err_prev		= 0.0f;
	priv->integral		= 0.0f;
	priv->diff_raw		= 0.0f;
	priv->diff_filtered	= 0.0f;
	priv->output		= 0.0f;
	priv->raw_output	= 0.0f;
	priv->feedback_last	= 0.0f;
	PID_UNLOCK(&priv->lock);
}

/**
 * pid_calc() - PID 核心运算
 * @priv:	PID 私有结构体
 * @fb:		当前反馈值
 * @ref:	目标值
 * @ref_dot:	目标一阶微分（速度），供速度前馈
 * @ref_ddot:	目标二阶微分（加速度），供加速度前馈
 * @dt:		控制周期，单位秒
 *
 * 返回限幅后的最终控制输出。
 */
static float pid_calc(struct pid_priv *priv, float fb, float ref,
		      float ref_dot, float ref_ddot, float dt)
{
	float pid_out   = 0.0f;
	float ff_out    = 0.0f;
	float final_out;
	bool  saturated = false;

	if (!priv)
		return 0.0f;

	/* dt 保护：必须在加锁之前，否则 early-return 时锁未释放 */
	if (dt <= 0.0f)
		return priv->output;

	PID_LOCK(&priv->lock);

	priv->feedback = fb;
	priv->setpoint = ref;
	priv->err      = priv->setpoint - priv->feedback;

	if (priv->mode == PID_MODE_POSITION) {
		/* 微分先行：对反馈微分，避免给定阶跃冲击 */
		priv->diff_raw = -(fb - priv->feedback_last) / dt;

		/* 不完全微分一阶低通 */
		priv->diff_filtered = priv->diff_alpha * priv->diff_raw
				    + (1.0f - priv->diff_alpha) * priv->diff_filtered;

		/* 积分累加（含 dt 因子） */
		priv->integral += priv->err * dt;

		pid_out = priv->kp * priv->err
			+ priv->ki * priv->integral
			+ priv->kd * priv->diff_filtered;
	} else {
		float delta_err = priv->err - priv->err_last;
		float delta_fb  = fb - priv->feedback_last;
		float delta_u;

		priv->diff_raw = -delta_fb / dt;
		priv->diff_filtered = priv->diff_alpha * priv->diff_raw
				    + (1.0f - priv->diff_alpha) * priv->diff_filtered;

		delta_u = priv->kp * delta_err
			+ priv->ki * priv->err * dt
			+ priv->kd * priv->diff_filtered;

		pid_out = priv->output + delta_u;
	}

	/* 保存未限幅本体输出（不含前馈），供饱和诊断 */
	priv->raw_output = pid_out;

	/* 叠加前馈 */
	if (priv->en_ffv)
		ff_out += priv->k_ffv * ref_dot;
	if (priv->en_ffa)
		ff_out += priv->k_ffa * ref_ddot;

	final_out = pid_out + ff_out;

	/* 输出限幅 */
	if (final_out > priv->out_max) {
		saturated = true;
		final_out = priv->out_max;
	} else if (final_out < priv->out_min) {
		saturated = true;
		final_out = priv->out_min;
	}

	/* 反计算抗积分饱和：只回退 PID 本体超量，前馈不参与 */
	if (saturated) {
		float pid_sat_err = 0.0f;

		if (pid_out > priv->out_max)
			pid_sat_err = pid_out - priv->out_max;
		else if (pid_out < priv->out_min)
			pid_sat_err = pid_out - priv->out_min;

		if (pid_sat_err != 0.0f)
			priv->integral -= priv->back_gain * pid_sat_err * dt;
	}

	priv->output = final_out;

	priv->err_prev      = priv->err_last;
	priv->err_last      = priv->err;
	priv->feedback_last = fb;

	PID_UNLOCK(&priv->lock);

	return final_out;
}

/**
 * pid_get_raw_output() - 返回最近一次 calc 的未限幅本体输出
 * @priv:	PID 私有结构体
 *
 * 未调用过 calc 或 priv 为空时返回 0。
 */
static float pid_get_raw_output(struct pid_priv *priv)
{
	if (!priv)
		return 0.0f;
	return priv->raw_output;
}

/* 默认 ops 接口绑定 */
static const struct pid_ops pid_default_ops = {
	.set_param	 = pid_set_param,
	.set_feedforward = pid_set_feedforward,
	.set_diff_filter = pid_set_diff_filter,
	.set_limit	 = pid_set_limit,
	.set_ref	 = pid_set_ref,
	.calc		 = pid_calc,
	.get_raw_output	 = pid_get_raw_output,
	.reset		 = pid_reset,
};

/*
 * pid_priv_defaults() - 公共默认参数初始化（init 使用）
 */
static void pid_priv_defaults(struct pid_priv *priv, enum pid_mode mode)
{
	pid_priv_clear(priv);

	PID_LOCK_INIT(&priv->lock);

	priv->mode = mode;
}

/**
 * pid_dev_init() - 用调用者提供的内存初始化 PID 实例
 *
 * 返回 0 成功，-1 参数非法（NULL 或 mode 越界）。
 */
int pid_dev_init(struct pid_dev *dev, struct pid_priv *priv, enum pid_mode mode)
{
	if (!dev || !priv)
		return -1;
	if (mode != PID_MODE_POSITION && mode != PID_MODE_INCREMENT)
		return -1;

	pid_priv_defaults(priv, mode);

	dev->priv = priv;
	dev->ops  = &pid_default_ops;

	return 0;
}

/**
 * pid_dev_deinit() - 反初始化静态实例
 *
 * 销毁锁，不释放内存。返回后句柄被清空。
 */
void pid_dev_deinit(struct pid_dev *dev)
{
	if (!dev || !dev->priv)
		return;

	PID_LOCK_DESTROY(&dev->priv->lock);

	dev->priv = NULL;
	dev->ops  = NULL;
}

#ifdef PID_USE_RTTHREAD
struct pid_dev *pid_dev_create(enum pid_mode mode)
{
	struct pid_dev  *dev;
	struct pid_priv *priv;

	if (mode != PID_MODE_POSITION && mode != PID_MODE_INCREMENT)
		return NULL;

	dev = (struct pid_dev *)rt_malloc(sizeof(*dev));
	if (!dev)
		return NULL;

	priv = (struct pid_priv *)rt_malloc(sizeof(*priv));
	if (!priv) {
		rt_free(dev);
		return NULL;
	}

	pid_priv_defaults(priv, mode);

	dev->priv = priv;
	dev->ops  = &pid_default_ops;

	return dev;
}

void pid_dev_destroy(struct pid_dev *dev)
{
	if (!dev)
		return;

	PID_LOCK_DESTROY(&dev->priv->lock);
	rt_free(dev->priv);
	rt_free(dev);
}
#endif /* PID_USE_RTTHREAD */
