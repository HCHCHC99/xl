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
	priv->i_term_max	= 0.0f;
	priv->aw_mask		= PID_AW_DEFAULT;
	priv->aw_cond_thr	= 0.0f;
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

static int pid_set_diff_filter(struct pid_priv *priv, float alpha)
{
	if (!priv)
		return -1;
	if (alpha < 0.0f || alpha > 1.0f)
		return -1;

	PID_LOCK(&priv->lock);
	priv->diff_alpha = alpha;
	PID_UNLOCK(&priv->lock);

	return 0;
}

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

static int pid_set_iterm_limit(struct pid_priv *priv, float i_term_max)
{
	if (!priv)
		return -1;
	if (i_term_max < 0.0f)
		return -1;

	PID_LOCK(&priv->lock);
	priv->i_term_max = i_term_max;
	PID_UNLOCK(&priv->lock);

	return 0;
}

static int pid_set_aw_mode(struct pid_priv *priv, uint32_t mask)
{
	if (!priv)
		return -1;
	if ((mask & ~(uint32_t)PID_AW_ALL) != 0u)
		return -1;

	PID_LOCK(&priv->lock);
	priv->aw_mask = mask;
	PID_UNLOCK(&priv->lock);

	return 0;
}

static int pid_set_aw_cond_thr(struct pid_priv *priv, float thr)
{
	if (!priv)
		return -1;
	if (thr < 0.0f)
		return -1;

	PID_LOCK(&priv->lock);
	priv->aw_cond_thr = thr;
	PID_UNLOCK(&priv->lock);

	return 0;
}

static int pid_set_ref(struct pid_priv *priv, float ref)
{
	if (!priv)
		return -1;

	PID_LOCK(&priv->lock);
	priv->setpoint = ref;
	PID_UNLOCK(&priv->lock);

	return 0;
}

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

static float pid_calc(struct pid_priv *priv, float fb, float ref,
		      float ref_dot, float ref_ddot, float dt)
{
	float pid_out   = 0.0f;
	float ff_out    = 0.0f;
	float final_out;
	float i_term    = 0.0f;
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

		/* 条件积分：上一拍本体输出已饱和且误差仍同向时冻结积分 */
		if ((priv->aw_mask & PID_AW_CONDITIONAL) && (priv->err != 0.0f)) {
			bool freeze = false;

			if ((priv->err > 0.0f) &&
			    (priv->raw_output >= (priv->out_max - priv->aw_cond_thr)))
				freeze = true;
			else if ((priv->err < 0.0f) &&
				 (priv->raw_output <= (priv->out_min + priv->aw_cond_thr)))
				freeze = true;

			if (!freeze)
				priv->integral += priv->err * dt;
		} else {
			priv->integral += priv->err * dt;
		}

		/* 钳位式抗饱和：I 项输出限幅，超限时反写 integral 保持连续 */
		i_term = priv->ki * priv->integral;
		if ((priv->aw_mask & PID_AW_CLAMP) &&
		    (priv->i_term_max > 0.0f) && (priv->ki > 0.0f)) {
			if (i_term > priv->i_term_max) {
				i_term = priv->i_term_max;
				priv->integral = i_term / priv->ki;
			} else if (i_term < -priv->i_term_max) {
				i_term = -priv->i_term_max;
				priv->integral = i_term / priv->ki;
			}
		}

		pid_out = priv->kp * priv->err
			+ i_term
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

	if (priv->en_ffv)
		ff_out += priv->k_ffv * ref_dot;
	if (priv->en_ffa)
		ff_out += priv->k_ffa * ref_ddot;

	final_out = pid_out + ff_out;

	if (final_out > priv->out_max) {
		saturated = true;
		final_out = priv->out_max;
	} else if (final_out < priv->out_min) {
		saturated = true;
		final_out = priv->out_min;
	}

	/* 反计算抗积分饱和：只回退 PID 本体超量，前馈不参与 */
	if (saturated && (priv->aw_mask & PID_AW_BACK_CALC)) {
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
	.set_iterm_limit = pid_set_iterm_limit,
	.set_aw_mode	 = pid_set_aw_mode,
	.set_aw_cond_thr = pid_set_aw_cond_thr,
	.set_ref	 = pid_set_ref,
	.calc		 = pid_calc,
	.get_raw_output	 = pid_get_raw_output,
	.reset		 = pid_reset,
};

static void pid_priv_defaults(struct pid_priv *priv, enum pid_mode mode)
{
	pid_priv_clear(priv);

	PID_LOCK_INIT(&priv->lock);

	priv->mode = mode;
}

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
