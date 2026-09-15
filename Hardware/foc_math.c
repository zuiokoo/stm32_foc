#include "foc_math.h"
#include <math.h>
#define INV_SQRT3 0.57735026919f
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

