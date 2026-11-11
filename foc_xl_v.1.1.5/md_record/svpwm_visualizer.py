"""
svpwm_visualizer.py - SVPWM 矢量分析工具（真实物理模型）

把 SguanFOC 库的 FOC 数学变换可视化，并用电机物理模型
+ 观测器模拟「角度观测误差 → 失步」的完整因果链:

    观测器误差(偏移/延迟/速度误差)
      → 程序角度 θ_est 偏差
      → V_ref(定子磁场)方向算错
      → 功角 δ = V_ref − θ_real 偏差
      → 电磁转矩 Te = Kt·Iq·sin(δ) 变小/反向
      → 转子转速跟不上/反转
      → 观测误差进一步拉大 → 正反馈失步

模型约定:
  * 电角度域 + 极对数 P=1 等效教学模型（失步机理与 P 无关）
  * 电流指令 q 为标么：0~1 对应 0~I_rated(A)

参数含义表:
  电机物理参数 (默认 3505-KV650):
    Kt      转矩常数 N·m/A = 1.5·P·Flux = 0.0126
    I_rated 额定电流 A = 17
    J       转动惯量 kg·m² = 7e-6 (估算)
    B       阻尼系数 N·m·s/rad = 4e-5 (估算)
    T_load  负载转矩 N·m = 0 (可加大演示失步)
  控制参数:
    d       匠轴电压标么 (默认 0 = id=0 控制)
    q       q轴电流标么 0~1 → 0~I_rated(A) (转矩指令)
  观测器参数 (模拟角度观测误差来源):
    delay_steps  采样延迟(步)
    offset_deg   常值角度偏移(°)  —— 对齐不准导致
    omega_error  速度观测误差(rad/s) —— 观测器速度偏差导致

运行:  python svpwm_visualizer.py
依赖:  Python 3 + tkinter
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import math
import sys
import os
import traceback
from collections import namedtuple

# ============ 全局常量（必须在所有类定义之前） ============
TWO_PI = 2.0 * math.pi

# 失步判定阈值
STABLE_DELTA    = 90.0      # 功角安全上限(°)
LOSE_SYNC_DELTA = 135.0     # 判定「发散中」的功角阈值(°)
FULL_LOST_DELTA = 160.0     # 判定「已失步」的功角阈值(°)

# 物理参数默认值 —— 3505-KV650 云台电机 (12V / 极对数 10)
# 模型约定: 电角度域 + P=1 等效教学模型（失步机理与极对数无关）
MOTOR_POLES  = 10        # 极对数(仅参考，模型不参与计算)
DEFAULT_KT   = 0.0126    # 转矩常数 N·m/A = 1.5·P·Flux = 1.5×10×0.00084
DEFAULT_I_RATED = 17.0   # 额定电流 A（q 输入为电流标么 0~1 → 0~17A）
DEFAULT_J    = 7e-6      # 转动惯量 kg·m² (估算: ½·m·r², 待实测)
DEFAULT_B    = 4e-5      # 阻尼系数 N·m·s/rad (估算: B≈Kt/omega, 待实测)
DEFAULT_T_LOAD = 0.0     # 负载转矩 N·m (失步演示可加到 ~0.1)
DEFAULT_DT   = 0.002     # 仿真步长 s（显式欧拉积分稳定性: dt < 2/wn ≈ 11ms）
SIM_INTERVAL_MS = 100    # 仿真步进间隔(ms)


def normalize_angle(a):
    """角度归一化到 (-π, π]"""
    a = math.fmod(a, TWO_PI)
    if a > math.pi:
        a -= TWO_PI
    elif a <= -math.pi:
        a += TWO_PI
    return a


class SguanMath:
    """Sguan_math.c 函数的Python实现"""
    
    @staticmethod
    def value_fabsf(x):
        return abs(x)
    
    @staticmethod
    def value_sqrtf(x):
        if x <= 0.0:
            return 0.0
        guess = x
        guess = (guess + x / guess) * 0.5
        guess = (guess + x / guess) * 0.5
        guess = (guess + x / guess) * 0.5
        return guess
    
    @staticmethod
    def value_limit(val, max_val, min_val):
        if val > max_val:
            return max_val
        if val < min_val:
            return min_val
        return val
    
    @staticmethod
    def fast_sin(x):
        pi = math.pi
        si = int(x * 0.31830988)
        x = x - si * pi
        if si & 1:
            x = x - pi if x > 0 else x + pi
        x2 = x * x
        f1 = 1.3528548e-10
        f1 = f1 * x2 + -2.4703144e-08
        f1 = f1 * x2 + 2.7532926e-06
        f1 = f1 * x2 + -0.00019840381
        f1 = f1 * x2 + 0.0083333179
        f1 = f1 * x2 + -0.16666666
        return x + x * x2 * f1
    
    @staticmethod
    def fast_cos(x):
        pi = math.pi
        si = int(x * 0.31830988)
        x = x - si * pi
        if si & 1:
            x = x - pi if x > 0 else x + pi
        x2 = x * x
        f2 = 1.7290616e-09
        f2 = f2 * x2 + -2.7093486e-07
        f2 = f2 * x2 + 2.4771643e-05
        f2 = f2 * x2 + -0.0013887906
        f2 = f2 * x2 + 0.041666519
        f2 = f2 * x2 + -0.49999991
        return 1.0 + x2 * f2
    
    @staticmethod
    def fast_sin_cos(x):
        return SguanMath.fast_sin(x), SguanMath.fast_cos(x)
    
    @staticmethod
    def clarke(i_a, i_b):
        inv_sqrt3 = 0.5773502691896257
        i_alpha = i_a
        i_beta = (i_a + 2 * i_b) * inv_sqrt3
        return i_alpha, i_beta
    
    @staticmethod
    def park(i_alpha, i_beta, sine, cosine):
        i_d = i_alpha * cosine + i_beta * sine
        i_q = i_beta * cosine - i_alpha * sine
        return i_d, i_q
    
    @staticmethod
    def ipark(u_d, u_q, sine, cosine):
        u_alpha = u_d * cosine - u_q * sine
        u_beta = u_q * cosine + u_d * sine
        return u_alpha, u_beta
    
    SvResult = namedtuple("SvResult", "d_u d_v d_w sector alpha beta")

    @staticmethod
    def svpwm(d, q, angle):
        pi = math.pi
        sqrt3 = 1.7320508075688772
        
        d = SguanMath.value_limit(d, 1, -1)
        q = SguanMath.value_limit(q, 1, -1)
        
        vref = d*d + q*q
        if vref > 1.0:
            scale = 1.0 / SguanMath.value_sqrtf(vref)
            d *= scale
            q *= scale
        
        sine, cosine = SguanMath.fast_sin_cos(angle)
        alpha = d * cosine - q * sine
        beta = q * cosine + d * sine
        
        A = 1 if beta > 0 else 0
        B = 1 if SguanMath.value_fabsf(beta) > sqrt3 * SguanMath.value_fabsf(alpha) else 0
        C = 1 if alpha > 0 else 0
        K = 4 * A + 2 * B + C
        K_to_sector = [4, 6, 5, 5, 3, 1, 2, 2]
        sector = K_to_sector[K]
        
        v = [[1,0,0], [1,1,0], [0,1,0], [0,1,1], [0,0,1], [1,0,1]]
        
        rad60 = 1.047197551196598
        angle0 = sector * rad60
        angle1 = angle0 - rad60
        
        sin_m, cos_m = SguanMath.fast_sin_cos(angle0)
        sin_n, cos_n = SguanMath.fast_sin_cos(angle1)
        
        t_m = sin_m * alpha - cos_m * beta
        t_n = beta * cos_n - alpha * sin_n
        t_0 = 1 - t_m - t_n
        
        idx = sector - 1
        idx_next = sector % 6
        
        d_u = t_m * v[idx][0] + t_n * v[idx_next][0] + t_0 / 2
        d_v = t_m * v[idx][1] + t_n * v[idx_next][1] + t_0 / 2
        d_w = t_m * v[idx][2] + t_n * v[idx_next][2] + t_0 / 2
        
        return SguanMath.SvResult(d_u, d_v, d_w, sector, alpha, beta)


class MotorModel:
    """电机物理模型 - 模拟真实电机的机械响应"""
    
    def __init__(self, J=DEFAULT_J, B=DEFAULT_B, Kt=DEFAULT_KT,
                 I_rated=DEFAULT_I_RATED, T_load=DEFAULT_T_LOAD):
        self.J = J
        self.B = B
        self.Kt = Kt
        self.I_rated = I_rated
        self.T_load = T_load
        
        self.theta = 0.0
        self.omega = 0.0
        self.current_direction = 1
    
    def reset(self, theta_0=0.0, omega_0=0.0):
        self.theta = theta_0
        self.omega = omega_0
        self.current_direction = 1 if omega_0 >= 0 else -1
    
    def step(self, V_ref_angle, iq, dt):
        """
        一步物理仿真
        V_ref_angle: 定子磁场方向 (rad)
        iq: q 轴电流 (A)，由电流标么 q × I_rated 得出
        dt: 仿真步长 (s)
        """
        # 计算功角 δ = V_ref 与真实 d轴 (θ) 的夹角
        delta = normalize_angle(V_ref_angle - self.theta)
        
        # 电磁转矩 Te = Kt · iq · sin(δ)：电流越大、功角越接近90°转矩越大
        Te = self.Kt * iq * math.sin(delta)
        
        # 机械运动方程: J * dω/dt = Te - T_load - B * ω
        domega = (Te - self.T_load - self.B * self.omega) * dt / self.J
        self.omega += domega
        
        # 更新位置
        self.theta += self.omega * dt
        
        # 更新方向
        if abs(self.omega) > 0.001:
            self.current_direction = 1 if self.omega > 0 else -1
        
        # 归一化到 [-π, π]
        self.theta = normalize_angle(self.theta)
        
        return self.theta, self.omega, Te, delta


class ObserverModel:
    """角度观测器 - 模拟程序估算的转子位置"""
    
    def __init__(self, delay_steps=0, offset_deg=0.0, omega_error=0.0):
        self.delay_steps = delay_steps
        self.offset = math.radians(offset_deg)
        self.omega_error = omega_error
        self.history = []
        self.drift = 0.0          # 角度漂移累积(由速度观测误差积分产生)
    
    def reset(self):
        self.history = []
        self.drift = 0.0
    
    def estimate(self, theta_real, omega_real, dt):
        """估算转子角度"""
        # 1. 恒定偏移
        theta_meas = theta_real + self.offset
        
        # 2. 速度观测误差 -> 角度漂移积累
        #    观测器认为每步走 omega_est*dt，真实走 omega_real*dt，
        #    差值 omega_error*dt 累积为角度偏差(模拟观测器积分漂移)
        self.drift += self.omega_error * dt
        
        # 3. 采样延迟
        self.history.append(theta_meas)
        if len(self.history) > self.delay_steps + 1:
            self.history.pop(0)
        
        if self.delay_steps > 0 and len(self.history) > self.delay_steps:
            theta_est = self.history[-(self.delay_steps + 1)]
        else:
            theta_est = theta_meas
        
        theta_est = normalize_angle(theta_est + self.drift)
        return theta_est


class SVPWMCanvas(tk.Canvas):
    """SVPWM画布"""
    
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(bg='#fafafa', highlightthickness=1, highlightbackground='#ccc')
        self.center_x = 250
        self.center_y = 250
        self.radius = 200
        self.rotor_radius = 0.6 * self.radius
        
        self.error_accumulated = 0.0
        self.step_count = 0
        self.is_diverging = False
        self.current_direction = 1
        
        self.current_data = None
        self.draw_hexagon()
    
    def draw_hexagon(self):
        self.delete("all")
        
        points = []
        for i in range(6):
            angle = i * math.pi / 3 - math.pi / 6
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y - self.radius * math.sin(angle)
            points.extend([x, y])
        
        self.create_polygon(points, outline='#333', fill='', width=2)
        
        self.create_line(self.center_x - self.radius - 20, self.center_y,
                        self.center_x + self.radius + 20, self.center_y,
                        fill='#999', dash=(3,3))
        self.create_line(self.center_x, self.center_y - self.radius - 20,
                        self.center_x, self.center_y + self.radius + 20,
                        fill='#999', dash=(3,3))
        
        self.create_text(self.center_x + self.radius + 30, self.center_y,
                        text='α', font=('Arial', 12, 'bold'), fill='#999')
        self.create_text(self.center_x, self.center_y - self.radius - 30,
                        text='β', font=('Arial', 12, 'bold'), fill='#999')
        
        for i in range(6):
            angle = i * math.pi / 3 + math.pi / 6
            x = self.center_x + (self.radius * 0.65) * math.cos(angle)
            y = self.center_y - (self.radius * 0.65) * math.sin(angle)
            self.create_text(x, y, text=f'{i+1}', font=('Arial', 14, 'bold'),
                           fill='#666')
        
        self.create_line(self.center_x, self.center_y,
                        self.center_x + self.radius * 1.05, self.center_y,
                        fill='#FF0000', width=2, dash=(4,4))
        self.create_text(self.center_x + self.radius * 1.1, self.center_y - 18,
                        text='A', font=('Arial', 12, 'bold'), fill='#FF0000')
        
        b_angle = 2 * math.pi / 3
        b_x = self.center_x + self.radius * 1.05 * math.cos(b_angle)
        b_y = self.center_y - self.radius * 1.05 * math.sin(b_angle)
        self.create_line(self.center_x, self.center_y, b_x, b_y,
                        fill='#009900', width=2, dash=(4,4))
        self.create_text(b_x + 15 * math.cos(b_angle),
                        b_y - 15 * math.sin(b_angle),
                        text='B', font=('Arial', 12, 'bold'), fill='#009900')
        
        c_angle = 4 * math.pi / 3
        c_x = self.center_x + self.radius * 1.05 * math.cos(c_angle)
        c_y = self.center_y - self.radius * 1.05 * math.sin(c_angle)
        self.create_line(self.center_x, self.center_y, c_x, c_y,
                        fill='#0000CC', width=2, dash=(4,4))
        self.create_text(c_x + 15 * math.cos(c_angle),
                        c_y - 15 * math.sin(c_angle),
                        text='C', font=('Arial', 12, 'bold'), fill='#0000CC')
    
    def draw_magnet(self, angle, length, width, color, label):
        center_r = self.rotor_radius * 0.75
        cx = self.center_x + center_r * math.cos(angle)
        cy = self.center_y - center_r * math.sin(angle)
        
        rad_x = math.cos(angle)
        rad_y = -math.sin(angle)
        tan_x = -math.sin(angle)
        tan_y = -math.cos(angle)
        
        half_len = length / 2
        half_wid = width / 2
        
        p1 = (cx + half_len * rad_x - half_wid * tan_x,
              cy + half_len * rad_y - half_wid * tan_y)
        p2 = (cx + half_len * rad_x + half_wid * tan_x,
              cy + half_len * rad_y + half_wid * tan_y)
        p3 = (cx - half_len * rad_x + half_wid * tan_x,
              cy - half_len * rad_y + half_wid * tan_y)
        p4 = (cx - half_len * rad_x - half_wid * tan_x,
              cy - half_len * rad_y - half_wid * tan_y)
        
        self.create_polygon([p1, p2, p3, p4], fill=color, outline='#333', width=1.5)
        
        label_x = cx + (half_len + 8) * rad_x
        label_y = cy + (half_len + 8) * rad_y
        self.create_text(label_x, label_y, text=label,
                        font=('Arial', 12, 'bold'), fill='white')
    
    def draw_vectors(self, d, q, theta_est, theta_real, alpha, beta, sector,
                     omega=0.0, Te=0.0, delta_deg=0.0,
                     error_accumulated=0.0, step_count=0, is_diverging=False,
                     current_direction=1):
        """绘制矢量图"""
        self.current_data = (d, q, theta_est, theta_real, alpha, beta, sector)
        self.current_direction = current_direction
        
        self.draw_hexagon()
        
        rotor_r = self.rotor_radius
        v_angle = math.atan2(beta, alpha)
        
        theta_est_deg = math.degrees(theta_est)
        theta_real_deg = math.degrees(theta_real)
        v_angle_deg = math.degrees(v_angle)
        omega_rpm = omega * 60 / (2 * math.pi)
        
        # ===== 转子 =====
        self.create_oval(self.center_x - rotor_r, self.center_y - rotor_r,
                        self.center_x + rotor_r, self.center_y + rotor_r,
                        outline='#555', width=2, fill='#E0E0E0')
        
        for r in [0.3, 0.6, 0.85]:
            rr = rotor_r * r
            self.create_oval(self.center_x - rr, self.center_y - rr,
                            self.center_x + rr, self.center_y + rr,
                            outline='#CCCCCC', width=0.5, fill='')
        
        magnet_len = rotor_r * 0.35
        magnet_wid = rotor_r * 0.20
        self.draw_magnet(theta_real, magnet_len, magnet_wid, '#FF4444', 'N')
        s_angle = theta_real + math.pi
        if s_angle > 2 * math.pi:
            s_angle -= 2 * math.pi
        self.draw_magnet(s_angle, magnet_len, magnet_wid, '#4488FF', 'S')
        
        self.create_oval(self.center_x - 5, self.center_y - 5,
                        self.center_x + 5, self.center_y + 5,
                        fill='#333', outline='#333')
        self.create_text(self.center_x, self.center_y + rotor_r + 20,
                        text='转子中心', font=('Arial', 8), fill='#555')
        
        # 真实d轴
        d_real_x = self.center_x + rotor_r * math.cos(theta_real)
        d_real_y = self.center_y - rotor_r * math.sin(theta_real)
        self.create_line(self.center_x, self.center_y, d_real_x, d_real_y,
                        fill='#9900CC', width=2.5, dash=(6,3))
        self.create_text(d_real_x + 15 * math.cos(theta_real),
                        d_real_y - 15 * math.sin(theta_real),
                        text='真实d轴', font=('Arial', 8), fill='#9900CC')
        
        # 程序d轴
        d_est_len = 0.65 * rotor_r
        d_est_x = self.center_x + d_est_len * math.cos(theta_est)
        d_est_y = self.center_y - d_est_len * math.sin(theta_est)
        self.create_line(self.center_x, self.center_y, d_est_x, d_est_y,
                        fill='#0066FF', width=2, dash=(6,4))
        self.create_text(d_est_x + 15 * math.cos(theta_est),
                        d_est_y - 15 * math.sin(theta_est),
                        text='程序d轴', font=('Arial', 8), fill='#0066FF')
        
        # Vref
        v_mag = math.sqrt(alpha*alpha + beta*beta)
        if v_mag > 0.001:
            v_len = min(v_mag, 1.0) * self.radius
            v_x = self.center_x + v_len * math.cos(v_angle)
            v_y = self.center_y - v_len * math.sin(v_angle)
            self.create_line(self.center_x, self.center_y, v_x, v_y,
                            fill='#FF8800', width=3)
            arrow_len = 10
            arrow_angle = 0.5
            self.create_polygon([v_x, v_y,
                                v_x - arrow_len * math.cos(v_angle - arrow_angle),
                                v_y + arrow_len * math.sin(v_angle - arrow_angle),
                                v_x - arrow_len * math.cos(v_angle + arrow_angle),
                                v_y + arrow_len * math.sin(v_angle + arrow_angle)],
                               fill='#FF8800', outline='#FF8800')
            self.create_text(v_x + 20 * math.cos(v_angle),
                            v_y - 20 * math.sin(v_angle),
                            text='V_ref', font=('Arial', 10, 'bold'), fill='#FF8800')
        
        # 受力点
        force_x = self.center_x + rotor_r * math.cos(theta_real)
        force_y = self.center_y - rotor_r * math.sin(theta_real)
        self.create_oval(force_x - 6, force_y - 6,
                        force_x + 6, force_y + 6,
                        fill='#FF0000', outline='#FF0000')
        self.create_text(force_x + 18, force_y - 18,
                        text='受力点', font=('Arial', 8, 'bold'), fill='#FF0000')
        
        # 切向力
        tan_x = -math.sin(theta_real)
        tan_y = -math.cos(theta_real)
        rad_x = math.cos(theta_real)
        rad_y = -math.sin(theta_real)
        
        F_t_mag = q
        F_r_mag = d
        
        if abs(F_t_mag) > 0.01:
            scale = 30
            ft_x = force_x + F_t_mag * scale * tan_x
            ft_y = force_y + F_t_mag * scale * tan_y
            
            if delta_deg > 0:
                ft_color = '#00AA00'
                ft_label = 'F_t (拉力)'
            else:
                ft_color = '#FF6600'
                ft_label = 'F_t (推力)'
            
            self.create_line(force_x, force_y, ft_x, ft_y,
                            fill=ft_color, width=3)
            ft_angle = math.atan2(-(ft_y - force_y), ft_x - force_x)
            arrow_len = 8
            self.create_polygon([ft_x, ft_y,
                                ft_x - arrow_len * math.cos(ft_angle - 0.5),
                                ft_y + arrow_len * math.sin(ft_angle - 0.5),
                                ft_x - arrow_len * math.cos(ft_angle + 0.5),
                                ft_y + arrow_len * math.sin(ft_angle + 0.5)],
                               fill=ft_color, outline=ft_color)
            self.create_text(ft_x + 15 * math.cos(ft_angle),
                            ft_y - 15 * math.sin(ft_angle),
                            text=ft_label, font=('Arial', 8), fill=ft_color)
        
        if abs(F_r_mag) > 0.01:
            scale = 25
            fr_x = force_x + F_r_mag * scale * rad_x
            fr_y = force_y + F_r_mag * scale * rad_y
            if F_r_mag > 0:
                fr_color = '#0066CC'
                fr_label = 'F_r (拉力)'
            else:
                fr_color = '#CC6600'
                fr_label = 'F_r (压力)'
            self.create_line(force_x, force_y, fr_x, fr_y,
                            fill=fr_color, width=2, dash=(4,3))
            fr_angle = math.atan2(-(fr_y - force_y), fr_x - force_x)
            arrow_len = 8
            self.create_polygon([fr_x, fr_y,
                                fr_x - arrow_len * math.cos(fr_angle - 0.5),
                                fr_y + arrow_len * math.sin(fr_angle - 0.5),
                                fr_x - arrow_len * math.cos(fr_angle + 0.5),
                                fr_y + arrow_len * math.sin(fr_angle + 0.5)],
                               fill=fr_color, outline=fr_color)
            self.create_text(fr_x + 15 * math.cos(fr_angle),
                            fr_y - 15 * math.sin(fr_angle),
                            text=fr_label, font=('Arial', 8), fill=fr_color)
        
        # 功角弧线
        if abs(delta_deg) > 0.2 and v_mag > 0.001:
            arc_radius = 45
            arc_points = []
            steps = 30
            start_a = theta_real
            end_a = v_angle
            if end_a - start_a > math.pi:
                end_a -= 2 * math.pi
            elif start_a - end_a > math.pi:
                end_a += 2 * math.pi
            for i in range(steps + 1):
                a = start_a + i * (end_a - start_a) / steps
                x = self.center_x + arc_radius * math.cos(a)
                y = self.center_y - arc_radius * math.sin(a)
                arc_points.extend([x, y])
            if len(arc_points) > 4:
                if abs(delta_deg) > 80:
                    arc_color = '#FF0000'
                elif abs(delta_deg) > 60:
                    arc_color = '#FF6600'
                else:
                    arc_color = '#00CC00'
                self.create_line(arc_points, fill=arc_color, width=2)
                mid_angle = (start_a + end_a) / 2
                label_radius = arc_radius + 20
                label_x = self.center_x + label_radius * math.cos(mid_angle)
                label_y = self.center_y - label_radius * math.sin(mid_angle)
                self.create_text(label_x, label_y,
                               text=f'δ={delta_deg:+.1f}°',
                               font=('Arial', 9, 'bold'), fill=arc_color)
        
        # 扇区
        sector_angle = sector * math.pi / 3 - math.pi / 6
        sx = self.center_x + (self.radius * 0.85) * math.cos(sector_angle + math.pi/6)
        sy = self.center_y - (self.radius * 0.85) * math.sin(sector_angle + math.pi/6)
        self.create_text(sx, sy,
                        text=f'扇区 {sector}',
                        font=('Arial', 12, 'bold'), fill='#CC0000')
        
        # ===== 状态显示 =====
        if delta_deg > 0:
            mode_text = "拉力模式 (V_ref 在前)"
            mode_color = "#00AA00"
        else:
            mode_text = "推力模式 (V_ref 在后)"
            mode_color = "#FF6600"
        
        dir_text = "正转" if current_direction == 1 else "反转"
        
        if abs(delta_deg) > 80:
            risk_text = "⚠️ 接近失步！"
            risk_color = "#FF0000"
        elif abs(delta_deg) > 60:
            risk_text = "⚡ 功角偏大"
            risk_color = "#FF6600"
        else:
            risk_text = "✅ 安全"
            risk_color = "#00AA00"
        
        status_text = f"{dir_text} | {mode_text} | δ={delta_deg:+.1f}° | {risk_text}"
        self.create_text(self.center_x,
                        self.center_y + self.radius * 0.78,
                        text=status_text,
                        font=('Arial', 10, 'bold'), fill=risk_color if abs(delta_deg)>60 else "#333")
        
        phys_text = f"ω={omega_rpm:.1f} RPM | Te={Te:.3f} N·m | 步数={step_count}"
        self.create_text(self.center_x,
                        self.center_y + self.radius * 0.86,
                        text=phys_text,
                        font=('Arial', 9), fill="#555")
        
        cumul_color = "#FF0000" if is_diverging else "#FF6600" if error_accumulated > 30 else "#00AA00"
        accum_text = f"角度误差: {error_accumulated:.1f}°"
        if is_diverging:
            accum_text += " ⚠️ 发散中！"
        
        self.create_text(self.center_x,
                        self.center_y + self.radius * 0.93,
                        text=accum_text,
                        font=('Arial', 9, 'bold'), fill=cumul_color)
        
        if is_diverging:
            self.create_text(self.center_x,
                            self.center_y + self.radius * 0.98,
                            text="💥 失步！",
                            font=('Arial', 10, 'bold'), fill="#FF0000")
    
    def update_vectors(self, d, q, theta_est, theta_real, alpha, beta, sector,
                       omega=0.0, Te=0.0, delta_deg=0.0,
                       error_accumulated=0.0, step_count=0, is_diverging=False,
                       current_direction=1):
        self.error_accumulated = error_accumulated
        self.step_count = step_count
        self.is_diverging = is_diverging
        self.draw_vectors(d, q, theta_est, theta_real, alpha, beta, sector,
                         omega, Te, delta_deg,
                         error_accumulated, step_count, is_diverging,
                         current_direction)


# ============ SVPWM测试应用 ============
class SVPWMApp:
    
    def __init__(self, root):
        self.root = root
        self.root.title("SVPWM 矢量分析工具 (真实物理模型)")
        self.root.geometry("1280x950")
        self.root.resizable(True, True)
        
        self.colors = {
            'bg': '#f0f2f5',
            'card_bg': '#ffffff',
            'primary': '#2c3e50',
            'accent': '#3498db',
            'success': '#27ae60',
            'warning': '#f39c12',
            'danger': '#e74c3c'
        }
        
        self.angle_unit = "deg"
        
        # 物理模型
        self.motor = MotorModel(J=DEFAULT_J, B=DEFAULT_B, Kt=DEFAULT_KT,
                             I_rated=DEFAULT_I_RATED, T_load=DEFAULT_T_LOAD)
        
        # 观测器
        self.observer = ObserverModel(delay_steps=0, offset_deg=0.0, omega_error=0.0)
        
        # 仿真状态
        self.error_accumulated = 0.0
        self.step_count = 0
        self.is_diverging = False
        self.sim_running = False
        self.sim_timer = None
        
        self.debug = os.environ.get("SGUAN_DEBUG", "") == "1"
        
        self.setup_ui()
        self.load_example()
    
    def log(self, msg):
        if self.debug:
            print(f"[DEBUG] {msg}")
    
    def setup_ui(self):
        self.log("开始创建UI...")
        
        main_frame = tk.Frame(self.root, bg=self.colors['bg'])
        main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        title_frame = tk.Frame(main_frame, bg=self.colors['bg'])
        title_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(title_frame, text="SVPWM 矢量分析 (真实物理模型)", 
                font=("Microsoft YaHei", 18, "bold"), 
                fg=self.colors['primary'], bg=self.colors['bg']).pack()
        
        tk.Label(title_frame, text="FOC控制层 + 电机物理模型 + 观测器 | Te = Kt·sin(δ) | J·dω/dt = Te - T_load - B·ω", 
                font=("Microsoft YaHei", 10), 
                fg="#7f8c8d", bg=self.colors['bg']).pack()
        
        body_frame = tk.Frame(main_frame, bg=self.colors['bg'])
        body_frame.pack(fill=tk.BOTH, expand=True)
        
        left_panel = tk.Frame(body_frame, bg=self.colors['card_bg'], 
                             relief=tk.GROOVE, bd=1)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        
        canvas_frame = tk.Frame(body_frame, bg=self.colors['bg'])
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # ===== 左侧面板 =====
        tk.Label(left_panel, text="参数控制", 
                font=("Microsoft YaHei", 13, "bold"),
                fg=self.colors['primary'], bg=self.colors['card_bg']).pack(pady=(15, 10))
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        param_frame = tk.Frame(left_panel, bg=self.colors['card_bg'])
        param_frame.pack(fill=tk.X, padx=20, pady=15)
        
        d_frame = tk.Frame(param_frame, bg=self.colors['card_bg'])
        d_frame.pack(fill=tk.X, pady=5)
        tk.Label(d_frame, text="d (励磁/径向)", font=("Microsoft YaHei", 10),
                fg="#555", bg=self.colors['card_bg'], width=12, anchor=tk.W).pack(side=tk.LEFT)
        self.d_entry = tk.Entry(d_frame, width=10, font=("Consolas", 11))
        self.d_entry.pack(side=tk.RIGHT)
        self.d_entry.insert(0, "0.0")
        
        q_frame = tk.Frame(param_frame, bg=self.colors['card_bg'])
        q_frame.pack(fill=tk.X, pady=5)
        tk.Label(q_frame, text="q (电流标么)", font=("Microsoft YaHei", 10),
                fg="#555", bg=self.colors['card_bg'], width=12, anchor=tk.W).pack(side=tk.LEFT)
        self.q_entry = tk.Entry(q_frame, width=10, font=("Consolas", 11))
        self.q_entry.pack(side=tk.RIGHT)
        self.q_entry.insert(0, "0.7")
        
        est_frame = tk.Frame(param_frame, bg=self.colors['card_bg'])
        est_frame.pack(fill=tk.X, pady=5)
        tk.Label(est_frame, text="θ_est (初始)", font=("Microsoft YaHei", 10),
                fg="#555", bg=self.colors['card_bg'], width=12, anchor=tk.W).pack(side=tk.LEFT)
        self.est_entry = tk.Entry(est_frame, width=10, font=("Consolas", 11))
        self.est_entry.pack(side=tk.RIGHT)
        self.est_entry.insert(0, "45.0")
        
        real_frame = tk.Frame(param_frame, bg=self.colors['card_bg'])
        real_frame.pack(fill=tk.X, pady=5)
        tk.Label(real_frame, text="θ_real (初始)", font=("Microsoft YaHei", 10),
                fg="#555", bg=self.colors['card_bg'], width=12, anchor=tk.W).pack(side=tk.LEFT)
        self.real_entry = tk.Entry(real_frame, width=10, font=("Consolas", 11))
        self.real_entry.pack(side=tk.RIGHT)
        self.real_entry.insert(0, "20.0")
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 观测器参数 =====
        obs_frame = tk.LabelFrame(left_panel, text="观测器参数 (模拟角度误差)", 
                                  font=("Microsoft YaHei", 10),
                                  bg=self.colors['card_bg'], fg=self.colors['primary'])
        obs_frame.pack(fill=tk.X, padx=15, pady=10)
        
        obs_param_frame = tk.Frame(obs_frame, bg=self.colors['card_bg'])
        obs_param_frame.pack(fill=tk.X, padx=10, pady=8)
        
        delay_frame = tk.Frame(obs_param_frame, bg=self.colors['card_bg'])
        delay_frame.pack(fill=tk.X, pady=3)
        tk.Label(delay_frame, text="采样延迟 (步):", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.delay_entry = tk.Entry(delay_frame, width=8, font=("Consolas", 10))
        self.delay_entry.pack(side=tk.RIGHT)
        self.delay_entry.insert(0, "0")
        
        offset_frame = tk.Frame(obs_param_frame, bg=self.colors['card_bg'])
        offset_frame.pack(fill=tk.X, pady=3)
        tk.Label(offset_frame, text="角度偏移 (°):", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.offset_entry = tk.Entry(offset_frame, width=8, font=("Consolas", 10))
        self.offset_entry.pack(side=tk.RIGHT)
        self.offset_entry.insert(0, "0.0")
        
        omega_err_frame = tk.Frame(obs_param_frame, bg=self.colors['card_bg'])
        omega_err_frame.pack(fill=tk.X, pady=3)
        tk.Label(omega_err_frame, text="速度误差 (rad/s):", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.omega_err_entry = tk.Entry(omega_err_frame, width=8, font=("Consolas", 10))
        self.omega_err_entry.pack(side=tk.RIGHT)
        self.omega_err_entry.insert(0, "0.0")
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 物理参数 =====
        phys_frame = tk.LabelFrame(left_panel, text="物理参数 (电机模型)", 
                                   font=("Microsoft YaHei", 10),
                                   bg=self.colors['card_bg'], fg=self.colors['primary'])
        phys_frame.pack(fill=tk.X, padx=15, pady=10)
        
        phys_param_frame = tk.Frame(phys_frame, bg=self.colors['card_bg'])
        phys_param_frame.pack(fill=tk.X, padx=10, pady=8)

        # J / B / Kt / I_rated 用科学计数法显示（可调）
        j_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        j_frame.pack(fill=tk.X, pady=3)
        tk.Label(j_frame, text="J 转动惯量:", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.j_entry = tk.Entry(j_frame, width=10, font=("Consolas", 10))
        self.j_entry.pack(side=tk.RIGHT)
        self.j_entry.insert(0, f"{DEFAULT_J:.2e}")

        b_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        b_frame.pack(fill=tk.X, pady=3)
        tk.Label(b_frame, text="B 阻尼系数:", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.b_entry = tk.Entry(b_frame, width=10, font=("Consolas", 10))
        self.b_entry.pack(side=tk.RIGHT)
        self.b_entry.insert(0, f"{DEFAULT_B:.2e}")

        kt_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        kt_frame.pack(fill=tk.X, pady=3)
        tk.Label(kt_frame, text="Kt 转矩常数:", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.kt_entry = tk.Entry(kt_frame, width=10, font=("Consolas", 10))
        self.kt_entry.pack(side=tk.RIGHT)
        self.kt_entry.insert(0, f"{DEFAULT_KT:.4f}")

        ir_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        ir_frame.pack(fill=tk.X, pady=3)
        tk.Label(ir_frame, text="I_rated 额定电流:", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.ir_entry = tk.Entry(ir_frame, width=10, font=("Consolas", 10))
        self.ir_entry.pack(side=tk.RIGHT)
        self.ir_entry.insert(0, f"{DEFAULT_I_RATED:.1f}")

        load_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        load_frame.pack(fill=tk.X, pady=3)
        tk.Label(load_frame, text="负载转矩 (N·m):", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.load_entry = tk.Entry(load_frame, width=8, font=("Consolas", 10))
        self.load_entry.pack(side=tk.RIGHT)
        self.load_entry.insert(0, "0.0")
        
        dt_frame = tk.Frame(phys_param_frame, bg=self.colors['card_bg'])
        dt_frame.pack(fill=tk.X, pady=3)
        tk.Label(dt_frame, text="仿真步长 (s):", font=("Microsoft YaHei", 9),
                fg="#555", bg=self.colors['card_bg'], width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.dt_entry = tk.Entry(dt_frame, width=8, font=("Consolas", 10))
        self.dt_entry.pack(side=tk.RIGHT)
        self.dt_entry.insert(0, "0.05")
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 单位选择 =====
        unit_frame = tk.Frame(left_panel, bg=self.colors['card_bg'])
        unit_frame.pack(fill=tk.X, padx=20, pady=8)
        tk.Label(unit_frame, text="角度单位:", font=("Microsoft YaHei", 10),
                fg="#555", bg=self.colors['card_bg']).pack(side=tk.LEFT)
        self.unit_var = tk.StringVar(value="deg")
        rad_btn = tk.Radiobutton(unit_frame, text="rad", variable=self.unit_var,
                                value="rad", command=self.on_unit_change,
                                font=("Microsoft YaHei", 9), bg=self.colors['card_bg'])
        rad_btn.pack(side=tk.LEFT, padx=5)
        deg_btn = tk.Radiobutton(unit_frame, text="°", variable=self.unit_var,
                                value="deg", command=self.on_unit_change,
                                font=("Microsoft YaHei", 9), bg=self.colors['card_bg'])
        deg_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 仿真控制 =====
        sim_frame = tk.LabelFrame(left_panel, text="仿真控制", 
                                  font=("Microsoft YaHei", 10),
                                  bg=self.colors['card_bg'], fg=self.colors['primary'])
        sim_frame.pack(fill=tk.X, padx=15, pady=10)
        
        sim_btn_frame = tk.Frame(sim_frame, bg=self.colors['card_bg'])
        sim_btn_frame.pack(fill=tk.X, padx=10, pady=8)
        
        self.sim_btn = tk.Button(sim_btn_frame, text="▶ 开始仿真", 
                                command=self.toggle_simulation,
                                font=("Microsoft YaHei", 9, "bold"),
                                bg=self.colors['success'], fg="white",
                                padx=15, pady=4, relief=tk.FLAT)
        self.sim_btn.pack(side=tk.LEFT, padx=3)
        
        self.reset_btn = tk.Button(sim_btn_frame, text="↺ 重置", 
                                  command=self.reset_simulation,
                                  font=("Microsoft YaHei", 9),
                                  bg="#95a5a6", fg="white",
                                  padx=15, pady=4, relief=tk.FLAT)
        self.reset_btn.pack(side=tk.LEFT, padx=3)
        
        self.accum_status = tk.Label(sim_frame, text="δ: 0.0° | ω: 0 RPM | Te: 0.00 N·m | 状态: 安全",
                                    font=("Microsoft YaHei", 9),
                                    fg="#00AA00", bg=self.colors['card_bg'])
        self.accum_status.pack(pady=5)
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 按钮 =====
        btn_frame = tk.Frame(left_panel, bg=self.colors['card_bg'])
        btn_frame.pack(fill=tk.X, padx=20, pady=10)
        btn_row = tk.Frame(btn_frame, bg=self.colors['card_bg'])
        btn_row.pack()
        
        self.btn_update = tk.Button(btn_row, text="▶ 执行计算", 
                                   command=self.execute,
                                   font=("Microsoft YaHei", 10, "bold"),
                                   bg=self.colors['success'], fg="white",
                                   padx=20, pady=5, relief=tk.FLAT)
        self.btn_update.pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_row, text="📋 示例", 
                 command=self.load_example,
                 font=("Microsoft YaHei", 10),
                 bg=self.colors['accent'], fg="white",
                 padx=15, pady=5, relief=tk.FLAT).pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(left_panel, orient='horizontal').pack(fill=tk.X, padx=15, pady=5)
        
        # ===== 结果 =====
        result_frame = tk.LabelFrame(left_panel, text="计算结果", 
                                     font=("Microsoft YaHei", 10),
                                     bg=self.colors['card_bg'], fg=self.colors['primary'])
        result_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(5, 15))
        
        self.result_text = scrolledtext.ScrolledText(result_frame, height=12,
                                                     font=("Consolas", 9),
                                                     bg="#fafafa", fg="#333",
                                                     relief=tk.FLAT, bd=1)
        self.result_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.result_text.config(state=tk.DISABLED)
        
        # ===== 右侧画布 =====
        legend_frame = tk.Frame(canvas_frame, bg=self.colors['bg'])
        legend_frame.pack(fill=tk.X, pady=(0, 5))
        legend_items = [
            ("A相", "#FF0000", "虚线"), ("B相", "#009900", "虚线"),
            ("C相", "#0000CC", "虚线"), ("V_ref", "#FF8800", "实线"),
            ("真实d轴", "#9900CC", "虚线"), ("程序d轴", "#0066FF", "虚线"),
            ("F_t", "#00AA00", "实线"), ("F_r", "#0066CC", "虚线"),
        ]
        for text, color, style in legend_items:
            if "虚线" in style:
                line = tk.Frame(legend_frame, width=20, height=2, bg=color)
                line.pack(side=tk.LEFT, padx=(2, 0))
            else:
                dot = tk.Frame(legend_frame, width=16, height=3, bg=color)
                dot.pack(side=tk.LEFT, padx=(2, 0))
            tk.Label(legend_frame, text=text, font=("Microsoft YaHei", 8),
                    fg="#555", bg=self.colors['bg']).pack(side=tk.LEFT, padx=(0, 8))
        
        self.canvas = SVPWMCanvas(canvas_frame, width=550, height=560)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # ===== 底部状态栏 =====
        status_frame = tk.Frame(main_frame, bg=self.colors['bg'])
        status_frame.pack(fill=tk.X, pady=(10, 0))
        self.status_bar = tk.Label(status_frame, text="就绪 — q>0 正力矩 | q<0 负力矩 | Te = Kt·sin(δ)",
                                  relief=tk.FLAT, anchor=tk.W,
                                  font=("Microsoft YaHei", 9),
                                  fg="#7f8c8d", bg=self.colors['bg'])
        self.status_bar.pack(fill=tk.X)
        
        self.log("UI创建全部完成！")
    
    def on_unit_change(self):
        unit = self.unit_var.get()
        try:
            for entry in [self.est_entry, self.real_entry]:
                val = float(entry.get())
                if unit == "deg":
                    new_val = math.degrees(val)
                else:
                    new_val = math.radians(val)
                entry.delete(0, tk.END)
                entry.insert(0, f"{new_val:.4f}")
        except:
            pass
        self.status_bar.config(text=f"单位切换为: {'度' if unit == 'deg' else '弧度'}")
    
    def load_example(self):
        self.log("加载示例...")
        
        self.d_entry.delete(0, tk.END)
        self.d_entry.insert(0, "0.0")
        self.q_entry.delete(0, tk.END)
        self.q_entry.insert(0, "0.7")
        self.est_entry.delete(0, tk.END)
        self.est_entry.insert(0, "45.0")
        self.real_entry.delete(0, tk.END)
        self.real_entry.insert(0, "20.0")
        self.delay_entry.delete(0, tk.END)
        self.delay_entry.insert(0, "0")
        self.offset_entry.delete(0, tk.END)
        self.offset_entry.insert(0, "0.0")
        self.omega_err_entry.delete(0, tk.END)
        self.omega_err_entry.insert(0, "0.0")
        self.j_entry.delete(0, tk.END)
        self.j_entry.insert(0, f"{DEFAULT_J:.2e}")
        self.b_entry.delete(0, tk.END)
        self.b_entry.insert(0, f"{DEFAULT_B:.2e}")
        self.kt_entry.delete(0, tk.END)
        self.kt_entry.insert(0, f"{DEFAULT_KT:.4f}")
        self.ir_entry.delete(0, tk.END)
        self.ir_entry.insert(0, f"{DEFAULT_I_RATED:.1f}")
        self.load_entry.delete(0, tk.END)
        self.load_entry.insert(0, "0.0")
        self.dt_entry.delete(0, tk.END)
        self.dt_entry.insert(0, f"{DEFAULT_DT:.4f}")
        
        self.reset_simulation()
        self.status_bar.config(text="已加载示例: q=0.7 (正力矩) · 初始功角 ≈ 25°")
        self.root.after(100, self.execute)
    
    def reset_simulation(self):
        """重置仿真状态"""
        self.step_count = 0
        self.is_diverging = False
        self.error_accumulated = 0.0
        
        try:
            theta_real_deg = float(self.real_entry.get())
            theta_real = math.radians(theta_real_deg)
        except:
            theta_real = 0.0

        # 从 UI 同步电机物理参数
        try:
            self.motor.J       = float(self.j_entry.get())
            self.motor.B       = float(self.b_entry.get())
            self.motor.Kt      = float(self.kt_entry.get())
            self.motor.I_rated = float(self.ir_entry.get())
            self.motor.T_load  = float(self.load_entry.get())
        except ValueError:
            pass

        # 从 UI 同步观测器参数（观测误差来源）
        try:
            self.observer.delay_steps = max(0, int(float(self.delay_entry.get())))
            self.observer.offset      = math.radians(float(self.offset_entry.get()))
            self.observer.omega_error = float(self.omega_err_entry.get())
        except ValueError:
            pass

        self.motor.reset(theta_real, 0.0)
        self.observer.reset()
        
        self.accum_status.config(
            text="δ: 0.0° | ω: 0 RPM | Te: 0.00 N·m | 状态: 安全",
            fg="#00AA00"
        )
        self.log("仿真已重置")
    
    def toggle_simulation(self):
        if self.sim_running:
            self.stop_simulation()
        else:
            self.start_simulation()
    
    def start_simulation(self):
        try:
            self.motor.T_load = float(self.load_entry.get())
            self.motor.J = DEFAULT_J
            self.motor.B = DEFAULT_B
            self.motor.Kt = DEFAULT_KT
        except:
            pass
        
        try:
            delay = int(self.delay_entry.get())
            offset = float(self.offset_entry.get())
            omega_err = float(self.omega_err_entry.get())
            self.observer.delay_steps = delay
            self.observer.offset = math.radians(offset)
            self.observer.omega_error = omega_err
        except:
            pass
        
        self.sim_running = True
        self.sim_btn.config(text="⏹ 停止仿真", bg="#e74c3c")
        self.status_bar.config(text="仿真运行中...")
        self.log("仿真开始")
        self.sim_step()
    
    def stop_simulation(self):
        self.sim_running = False
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None
        self.sim_btn.config(text="▶ 开始仿真", bg=self.colors['success'])
        self.status_bar.config(text="仿真已停止")
        self.log("仿真停止")
    
    def sim_step(self):
        """一帧仿真 - 真实物理模型（每帧多步积分）

        每个动画帧(SIM_INTERVAL_MS)内积分 steps 步:
            steps = SIM_INTERVAL_MS/1000/dt
        这样 dt 可以用真实的小步长(2ms)保证积分稳定，
        动画节奏仍保持在 100ms/帧。
        """
        if not self.sim_running:
            return
        
        try:
            dt = float(self.dt_entry.get())
            d = float(self.d_entry.get())
            q = float(self.q_entry.get())
            iq = q * self.motor.I_rated          # 电流标么 q(0~1) → 实际电流 A
            steps = max(1, round(SIM_INTERVAL_MS / 1000.0 / dt))  # 每帧积分步数

            # 每帧同步观测器参数（UI 可实时调）
            try:
                self.observer.delay_steps = max(0, int(float(self.delay_entry.get())))
                self.observer.offset      = math.radians(float(self.offset_entry.get()))
                self.observer.omega_error = float(self.omega_err_entry.get())
            except ValueError:
                pass

            for _ in range(steps):
                # 1. 当前真实转子位置/速度
                theta_real = self.motor.theta
                omega_real = self.motor.omega

                # 2. 观测器估算角度
                theta_est = self.observer.estimate(theta_real, omega_real, dt)

                # 3. FOC层: V_ref
                result = SguanMath.svpwm(d, q, theta_est)
                V_ref_angle = math.atan2(result.beta, result.alpha)

                # 4. 电机物理模型一步
                theta_new, omega_new, Te, delta = self.motor.step(V_ref_angle, iq, dt)

                # 5. 状态
                delta_deg = math.degrees(delta)
                error_deg = math.degrees(normalize_angle(theta_est - theta_new))
                self.error_accumulated = abs(error_deg)
                self.step_count += 1

                # 6. 失步检测
                if abs(delta_deg) > FULL_LOST_DELTA:
                    self.is_diverging = True
                    self.status_bar.config(text="💥 已失步 |δ|>160°")
                    self._update_display(d, q, theta_est, theta_new,
                                        result.alpha, result.beta, result.sector,
                                        omega_new, Te, delta_deg)
                    self.stop_simulation()
                    return
                if abs(delta_deg) > LOSE_SYNC_DELTA:
                    self.is_diverging = True

            # 7. 更新显示（只用最后一步的结果）
            self._update_display(d, q, theta_est, theta_new,
                                result.alpha, result.beta, result.sector,
                                omega_new, Te, delta_deg)

            # 8. 继续下一帧
            self.sim_timer = self.root.after(SIM_INTERVAL_MS, self.sim_step)

        except Exception as e:
            self.log(f"仿真步进错误: {e}")
            traceback.print_exc()
            self.stop_simulation()
    
    def _update_display(self, d, q, theta_est, theta_real, alpha, beta, sector,
                        omega, Te, delta_deg):
        """更新显示"""
        # 更新输入框（显示角度在 0~360° 范围内）
        est_deg = math.degrees(theta_est) % 360
        real_deg = math.degrees(theta_real) % 360
        
        self.est_entry.delete(0, tk.END)
        self.est_entry.insert(0, f"{est_deg:.2f}")
        self.real_entry.delete(0, tk.END)
        self.real_entry.insert(0, f"{real_deg:.2f}")
        
        # 更新画布
        self.canvas.update_vectors(
            d, q, theta_est, theta_real, alpha, beta, sector,
            omega, Te, delta_deg,
            self.error_accumulated, self.step_count, self.is_diverging,
            self.motor.current_direction
        )
        
        # 更新状态显示
        omega_rpm = omega * 60 / (2 * math.pi)
        if self.is_diverging:
            color, status = "#FF0000", "发散中！"
        elif abs(delta_deg) > STABLE_DELTA:
            color, status = "#FF6600", "功角偏大"
        else:
            color, status = "#00AA00", "安全"
        
        self.accum_status.config(
            text=f"δ={delta_deg:+.1f}° | ω={omega_rpm:.1f} RPM | Te={Te:.3f} N·m | {status}",
            fg=color
        )
        
        dir_text = "正转" if omega > 0.01 else "反转" if omega < -0.01 else "停止"
        self.status_bar.config(
            text=f"仿真中 | {dir_text} | δ={delta_deg:+.1f}° | ω={omega_rpm:.1f} RPM | Te={Te:.3f} N·m"
        )
    
    def execute(self):
        """执行单次计算（静态显示）"""
        self.log("=" * 50)
        self.log("执行计算开始...")
        
        try:
            d = float(self.d_entry.get().strip())
            q = float(self.q_entry.get().strip())
            theta_est_deg = float(self.est_entry.get().strip())
            theta_real_deg = float(self.real_entry.get().strip())
            
            if self.unit_var.get() == "deg":
                theta_est = math.radians(theta_est_deg)
                theta_real = math.radians(theta_real_deg)
            else:
                theta_est = theta_est_deg
                theta_real = theta_real_deg
            
            result = SguanMath.svpwm(d, q, theta_est)
            V_ref_angle = math.atan2(result.beta, result.alpha)
            
            delta = normalize_angle(V_ref_angle - theta_real)
            delta_deg = math.degrees(delta)
            Te = math.sin(delta)
            
            self.canvas.update_vectors(
                d, q, theta_est, theta_real,
                result.alpha, result.beta, result.sector,
                0.0, Te, delta_deg,
                self.error_accumulated, self.step_count, self.is_diverging,
                1
            )
            
            self.update_result(d, q, theta_est, theta_real,
                             result.alpha, result.beta, result.sector,
                             result.d_u, result.d_v, result.d_w,
                             V_ref_angle, delta, Te)
            
        except ValueError as e:
            self.log(f"ValueError: {e}")
            self.status_bar.config(text="输入错误")
        except Exception as e:
            self.log(f"执行错误: {e}")
            self.log(traceback.format_exc())
            self.status_bar.config(text=f"错误: {str(e)}")
        
        self.log("执行计算结束")
        self.log("=" * 50)
    
    def update_result(self, d, q, theta_est, theta_real, alpha, beta, sector,
                     d_u, d_v, d_w, v_angle, delta, Te):
        try:
            self.result_text.config(state=tk.NORMAL)
            self.result_text.delete(1.0, tk.END)
            
            theta_est_deg = math.degrees(theta_est)
            theta_real_deg = math.degrees(theta_real)
            v_angle_deg = math.degrees(v_angle)
            delta_deg = math.degrees(delta)
            
            lines = []
            lines.append("═" * 50)
            lines.append(" 输入参数")
            lines.append("─" * 50)
            lines.append(f"  d = {d:+.4f}  (励磁/径向)")
            lines.append(f"  q = {q:+.4f}  (力矩指令)")
            lines.append(f"  θ_est      → {theta_est_deg:+.1f}°")
            lines.append(f"  θ_real     → {theta_real_deg:+.1f}°")
            lines.append("")
            lines.append("─" * 50)
            lines.append(" FOC 输出")
            lines.append("─" * 50)
            lines.append(f"  扇区     → {sector}")
            lines.append(f"  d_u      → {d_u:.4f}  ({d_u*100:.1f}%)")
            lines.append(f"  d_v      → {d_v:.4f}  ({d_v*100:.1f}%)")
            lines.append(f"  d_w      → {d_w:.4f}  ({d_w*100:.1f}%)")
            lines.append("")
            lines.append("─" * 50)
            lines.append(" 物理量")
            lines.append("─" * 50)
            lines.append(f"  V_ref    → {v_angle_deg:+.1f}° (定子磁场)")
            lines.append(f"  真实d轴  → {theta_real_deg:+.1f}° (N极)")
            lines.append(f"  δ = V_ref - d轴 = {delta_deg:+.1f}°")
            lines.append(f"  Te = Kt·sin(δ) = {Te:.4f} N·m")
            
            if delta > 0:
                lines.append(f"  ✅ 拉力模式 (V_ref 在 N极 前方)")
            else:
                lines.append(f"  ❌ 推力模式 (V_ref 在 N极 后方)")
            
            if abs(delta_deg) > 80:
                lines.append("")
                lines.append("  ⚠️ 严重：功角过大，接近失步！")
            elif abs(delta_deg) > 60:
                lines.append("")
                lines.append("  ⚡ 警告：功角偏大，效率降低")
            else:
                lines.append("")
                lines.append("  ✅ 功角安全")
            
            lines.append("═" * 50)
            
            output = "\n".join(lines)
            self.result_text.insert(1.0, output)
            self.result_text.config(state=tk.DISABLED)
        except Exception as e:
            self.log(f"update_result错误: {e}")


# ============ 独立运行入口 ============
if __name__ == "__main__":
    print("=" * 60)
    print("启动 SVPWM 可视化工具 (真实物理模型)...")
    print(f"Python 版本: {sys.version}")
    print("=" * 60)
    
    try:
        root = tk.Tk()
        print("Tkinter 根窗口创建成功")
        
        app = SVPWMApp(root)
        print("SVPWMApp 实例创建成功")
        
        print("进入主事件循环...")
        print("=" * 60)
        root.mainloop()
        print("程序正常退出")
    except Exception as e:
        print(f"程序启动失败: {e}")
        traceback.print_exc()
        input("\n按 Enter 键退出...")