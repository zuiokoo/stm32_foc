#include "foc_math.h"
#include <math.h>
#define INV_SQRT3 0.57735026919f

/* ---- 电流采样窗口约束（与硬件时序绑定，改 PWM/ADC 配置时必须同步更新）----
 * PWM    : TIM1 中心对齐，ARR = 2100-1，定时器时钟 84 MHz → 周期 50 us
 * 注入组 : 2 通道 × 84 周期采样，ADC 时钟 21 MHz
 *          → 单次转换 (84+12) = 96 周期，两路 192 周期 ≈ 9.1 us
 *  再留 1.5 us 余量（开关沿、ADC 触发抖动）→ 需要的全低窗口 10.6 us
 * 详细推导见 foc_svpwm() 中的注释。 */
#define FOC_PWM_PERIOD_S       (50.0e-6f)
#define FOC_INJ_WINDOW_NEED_S  (10.6e-6f)

void  foc_clarke_transform(float iu_a,float iv_a,float *i_alpha_a,float *i_beta_a)
{

	*i_alpha_a = iu_a;
    *i_beta_a = (iu_a + 2.0f * iv_a) * INV_SQRT3;

}
void  foc_park_transform(float i_alpha_a,float i_beta_a,float electrical_angle_rad,float *i_d_a,float *i_q_a)
{
	float sin_angle = sinf(electrical_angle_rad);
	float cos_angle = cosf(electrical_angle_rad);

	*i_d_a = i_alpha_a * cos_angle + i_beta_a * sin_angle;
	*i_q_a = -i_alpha_a * sin_angle + i_beta_a * cos_angle;
}

void  foc_inverse_park_transform(float v_d_v,float v_q_v,float electrical_angle_rad,float *v_alpha_v,float *v_beta_v)
{

	float sin_angle = sinf(electrical_angle_rad);
	float cos_angle = cosf(electrical_angle_rad);

	*v_alpha_v = v_d_v * cos_angle - v_q_v * sin_angle;
	*v_beta_v = v_d_v * sin_angle + v_q_v * cos_angle;

}
void foc_inverse_clarke_transform(float v_alpha_v,float v_beta_v,float *u_voltage_v,float *v_voltage_v,float *w_voltage_v)
{
	const float sqrt_3_half=0.8660254f;
	*u_voltage_v=v_alpha_v;
	*v_voltage_v=-0.5f*v_alpha_v+sqrt_3_half*v_beta_v;
	*w_voltage_v=-0.5f*v_alpha_v-sqrt_3_half*v_beta_v;
}
float foc_mechanical_to_electrical_angle(float mechanical_angle_rad,int pole_pairs,float electrical_zero_offset_rad)
{
	// 一整圈角度，单位为 rad。
	const float two_pi = 2.0f * 3.14159265358979323846f;
	/*
	 * 电角度关系：
	 *
	 * 电角度 = 机械角度 × 极对数 - 电角度零点偏移
	 */
	float electrical_angle_rad = mechanical_angle_rad * pole_pairs - electrical_zero_offset_rad;
	/*
	 * 将电角度限制到 0 ~ 2π。
	 * fmodf() 可以取得浮点数除法的余数。
	 */
	electrical_angle_rad = fmodf(electrical_angle_rad, two_pi);
	// fmodf() 对负数可能返回负数，所以补回一整圈。
	if (electrical_angle_rad < 0.0f)
	{
		electrical_angle_rad += two_pi;
	}
	return electrical_angle_rad;
}

static float foc_clamp(float value,float min_value,float max_value){
    if(value<min_value){
        return min_value;
    }
    if(value>max_value){
        return max_value;
    }
    return value;
}

uint8_t  foc_svpwm(float v_alpha_v,float v_beta_v,float vbus_v,float *duty_u,float *duty_v,float * duty_w){
    
    float u_voltage;
    float v_voltage;
    float w_voltage;
    
    float max_voltage;
    float min_voltage;
    float common_voltage;
    float span_voltage;
    float span_limit;
    float span_scale;
        
    if (vbus_v <= 0.0f)
    {
         *duty_u = 0.5f;
         *duty_v = 0.5f;
         *duty_w = 0.5f;
         return 0;
    }
    
    foc_inverse_clarke_transform(v_alpha_v,v_beta_v,&u_voltage,&v_voltage,&w_voltage);
    max_voltage = u_voltage;
    if(v_voltage >max_voltage){
    
         max_voltage = v_voltage;
    }
    if (w_voltage > max_voltage)
    {
        max_voltage = w_voltage;
    }
        min_voltage = u_voltage;

    if (v_voltage < min_voltage)
    {
        min_voltage = v_voltage;
    }

    if (w_voltage < min_voltage)
    {
        min_voltage = w_voltage;
    }

    /* ---- 低端分流采样窗口限幅 ----
     * 加共模电压后三相占空比围绕 0.5 对称，所以
     *     最大占空比 = 0.5 + 跨度/(2·Vbus)，其中 跨度 = max - min
     * "三路全低"（零矢量）窗口 = (1 - 最大占空比) × PWM周期
     *                        = (PWM周期/2) × (1 - 跨度/Vbus)
     * 该窗口必须装得下两路注入转换，于是：
     *     跨度 ≤ Vbus × (1 - 2·FOC_INJ_WINDOW_NEED_S/FOC_PWM_PERIOD_S)
     *          ≈ 0.576 × Vbus      （12V 母线 → 6.91V）
     * 跨度相对于矢量幅值随电角度在 1.5 ~ √3 倍之间变化，故
     *     等效可用矢量幅值 ≈ 4.6V（跨度为 1.5 倍时）~ 4.0V（跨度为 √3 倍时）
     * 注意：这里削的是"实际施加"的电压；motor2_vd/vq 仍是 PI 的需求值，
     * 所以 VOFA 上看 vd/vq 看不出限幅，要观察电流/转速是否还跟得上需求。
     * 闭环被频繁限幅时，PI 可能积分饱和，届时应把 PI 输出限幅也收到这个上限。 */
    span_voltage = max_voltage - min_voltage;
    span_limit = vbus_v * (1.0f - 2.0f * FOC_INJ_WINDOW_NEED_S / FOC_PWM_PERIOD_S);
    if (span_voltage > span_limit)
    {
        span_scale = span_limit / span_voltage;
        u_voltage *= span_scale;
        v_voltage *= span_scale;
        w_voltage *= span_scale;
        max_voltage *= span_scale;
        min_voltage *= span_scale;
    }

    common_voltage=-0.5f*(max_voltage +min_voltage);
    u_voltage += common_voltage;
    v_voltage += common_voltage;
    w_voltage += common_voltage;
    
    *duty_u=0.5f+u_voltage/vbus_v;
    *duty_v=0.5f+v_voltage/vbus_v;
    *duty_w=0.5f+w_voltage/vbus_v;
    
    *duty_u=foc_clamp(*duty_u,0.0f,1.0f);
    *duty_v=foc_clamp(*duty_v,0.0f,1.0f);
    *duty_w=foc_clamp(*duty_w,0.0f,1.0f);
    
    return 1;

}

