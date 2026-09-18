#include "vofa.h"
#include "usart.h"

//extern  volatile  uint8_t vofa_enable;
//extern  volatile uint8_t vofa_send_flag;

extern volatile float motor2_electrical_angle_rad;
extern volatile float motor2_id_ref;
extern volatile float motor2_iq_ref;
extern  float motor2_id;
extern  float motor2_iq;
extern  float motor2_vd;
extern  float motor2_vq;



void vofa_init(vofa_t*vofa){
    vofa->vofa_send_flag=0;
    vofa->vofa_enable=0;
    for(uint8_t i=0;i<VOFA_DATA_NUM;i++)
    {
        vofa->vofa_data[i]=0;
    }
}

void vofa_send(float*data,uint8_t num){

    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart2, (uint8_t *)data, num * 4, 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)tail, 4, 100);
    
}

void vofa_capture(float* data)
{
    data[0] = motor2_electrical_angle_rad;
    data[1] = motor2_id_ref;
    data[2] = motor2_id;
    data[3] = motor2_iq_ref;
    data[4] = motor2_iq;
    data[5] = motor2_vd;
    data[6] = motor2_vq;
}



