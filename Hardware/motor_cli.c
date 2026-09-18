#include "motor_cli.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "pi_controller.h"
#include "vofa.h"

typedef enum {
    CLI_TX_NONE = 0,
    CLI_TX_ALIGN_OK,
    CLI_TX_RUN_OK,
    CLI_TX_STOP_OK,
    CLI_TX_ERR,
    CLI_TX_VOFA_ON,
    CLI_TX_VOFA_OFF,
    CLI_TX_HELP,
    CLI_TX_STATUS,
    CLI_TX_PID_OK,
    CLI_TX_REF_OK,
} cli_tx_msg_t;
extern UART_HandleTypeDef huart2;

extern volatile uint8_t motor2_align_request;
extern volatile uint8_t motor2_run_enable;


extern volatile float motor2_id_ref;
extern volatile float motor2_iq_ref;


extern foc_pi_t motor2_pi_d;
extern foc_pi_t motor2_pi_q;


extern float motor2_id;
extern float motor2_iq;

extern volatile float motor2_electrical_angle_rad;

extern uint8_t uart2_rx_byte;

extern vofa_t motor2_vofa;

extern volatile float motor2_openloop_angle;
extern volatile float motor2_openloop_speed;
extern volatile float motor2_openloop_voltage;
extern volatile uint8_t motor2_openloop_enable;

#define CLI_TX_BUF_SIZE 160
#define CLI_TXQ_SIZE     8       /* 发送队列深度，实际可存 7 条 */

#define CLI_BUF_SIZE 64
static char cli_buf[CLI_BUF_SIZE];   // ← 接收缓冲，存 PC 发来的命令
static uint8_t cli_index = 0;

/* 发送队列：环形 */
static volatile cli_tx_msg_t cli_txq[CLI_TXQ_SIZE];
static volatile uint8_t      cli_txq_head = 0;   /* 中断写 */
static volatile uint8_t      cli_txq_tail = 0;   /* 主循环读 */

//入队（中断里调用）
static void cli_enqueue(cli_tx_msg_t msg){
    
    uint8_t next=(cli_txq_head+1)%CLI_TXQ_SIZE;
    if(next==cli_txq_tail){
        return;/* 队列满，丢弃 */
    }
    cli_txq[cli_txq_head]=msg;
    cli_txq_head=next;
}

void motor_cli_poll(void)
{
    while (cli_txq_tail != cli_txq_head) {

        cli_tx_msg_t msg = cli_txq[cli_txq_tail];
        cli_txq_tail = (cli_txq_tail + 1) % CLI_TXQ_SIZE;

        char buf[CLI_TX_BUF_SIZE];

        switch (msg) {
        case CLI_TX_ALIGN_OK:
            strcpy(buf, "ALIGN OK\r\n");
            break;
        case CLI_TX_RUN_OK:
            strcpy(buf, "RUN OK\r\n");
            break;
        case CLI_TX_STOP_OK:
            strcpy(buf, "STOP OK\r\n");
            break;
        case CLI_TX_ERR:
            strcpy(buf, "ERR\r\n");
            break;
        case CLI_TX_VOFA_ON:
            strcpy(buf, "VOFA ON OK\r\n");
            break;
        case CLI_TX_VOFA_OFF:
            strcpy(buf, "VOFA OFF OK\r\n");
            break;
        case CLI_TX_PID_OK:
            strcpy(buf, "PID OK\r\n");
            break;
        case CLI_TX_REF_OK:
            strcpy(buf, "REF OK\r\n");
            break;
        case CLI_TX_HELP:
            strcpy(buf, "CMD:\r\n"
                        "ALIGN\r\n"
                        "RUN\r\n"
                        "STOP\r\n"
                        "ID x\r\n"
                        "IQ x\r\n"
                        "PID_D kp ki\r\n"
                        "PID_Q kp ki\r\n"
                        "STATUS\r\n"
                        "VOFA ON\r\n"
                        "VOFA OFF\r\n"
                        "HELP\r\n");
            break;
        case CLI_TX_STATUS:
            snprintf(buf, sizeof(buf),
                     "STATE\r\n"
                     "ANGLE %.3f\r\n"
                     "ID %.3f\r\n"
                     "IQ %.3f\r\n"
                     "ID_REF %.3f\r\n"
                     "IQ_REF %.3f\r\n",
                     motor2_electrical_angle_rad,
                     motor2_id, motor2_iq,
                     motor2_id_ref, motor2_iq_ref);
            break;
        default:
            continue;           /* 未知枚举，跳过 */
        }

        HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
    }
}
void motor_cli_init(void){

    cli_index = 0;
    cli_txq_head=0;
    cli_txq_tail=0;
    memset(cli_buf, 0, sizeof(cli_buf));
    memset((void*)cli_txq,0,sizeof(cli_txq));
}


void motor_cli_parse(char*cmd){
    float v1,v2;
    if(strcmp(cmd,"ALIGN")==0){
        motor2_align_request=1;
        cli_enqueue(CLI_TX_ALIGN_OK);

    }
    else if(strcmp(cmd,"RUN")==0){
        motor2_run_enable=1;
        cli_enqueue (CLI_TX_RUN_OK);

    }
    else if(strcmp(cmd,"STOP")==0){
        motor2_run_enable=0;
        cli_enqueue (CLI_TX_STOP_OK);
 
    }
    else if(sscanf(cmd,"ID %f",&v1)==1){
        motor2_id_ref=v1;
        cli_enqueue(CLI_TX_REF_OK);
    }
    else if(sscanf(cmd,"IQ %f",&v1)==1)
    {
        motor2_iq_ref=v1;
        cli_enqueue(CLI_TX_REF_OK);
    }
    else if(sscanf(cmd,"PID_D %f %f",&v1,&v2)==2){
        motor2_pi_d.kp=v1;
        motor2_pi_d.ki=v2;
        cli_enqueue (CLI_TX_PID_OK);
    }
    else if(sscanf(cmd,"PID_Q %f %f",&v1,&v2)==2){
        motor2_pi_q.kp=v1;
        motor2_pi_q.ki=v2;
        cli_enqueue (CLI_TX_PID_OK);
    } 
    else if(strcmp(cmd,"STATUS")==0){
        cli_enqueue (CLI_TX_STATUS);

    }
    else if(strcmp(cmd,"HELP")==0)
    {
        cli_enqueue (CLI_TX_HELP);

    }
    else if(strcmp(cmd,"VOFA ON")==0){
        motor2_vofa.vofa_enable=1;
        cli_enqueue (CLI_TX_VOFA_ON);

    }
    else if(strcmp(cmd,"VOFA OFF")==0){
        motor2_vofa.vofa_enable=0;
        cli_enqueue (CLI_TX_VOFA_OFF);
 
    }
    else if(strcmp(cmd,"OPENLOOP")==0){
    motor2_openloop_enable = 1;
    cli_enqueue(CLI_TX_RUN_OK);
}
else if(strcmp(cmd,"CLOSELOOP")==0){
    motor2_openloop_enable = 0;
    cli_enqueue(CLI_TX_STOP_OK);
}
else if(sscanf(cmd,"OL_SPEED %f",&v1)==1){
    motor2_openloop_speed = v1;
    cli_enqueue(CLI_TX_REF_OK);
}
else if(sscanf(cmd,"OL_VOLT %f",&v1)==1){
    motor2_openloop_voltage = v1;
    cli_enqueue(CLI_TX_REF_OK);
}
    else
    {
        cli_enqueue(CLI_TX_ERR);
    }
            
}

void motor_cli_rx_char(uint8_t ch){


    if(ch=='\r'||ch=='\n'){
        if(cli_index){
            cli_buf[cli_index]=0;
//            printf("CMD=[%s]\r\n",cli_buf);
            motor_cli_parse(cli_buf);
            cli_index=0;
            memset(cli_buf,0,sizeof(cli_buf));
        }
    }
    else{
    
        if(cli_index<CLI_BUF_SIZE-1){
        
            cli_buf[cli_index++]=ch;
        }
    
    }

}
