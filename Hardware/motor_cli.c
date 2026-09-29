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
    CLI_TX_CLEAR_OK,
    CLI_TX_CAP_OK,
    CLI_TX_SPD_OK,
    CLI_TX_SPD_OFF_OK,
    CLI_TX_CAPS_OK,
    CLI_TX_POS_OK,
} cli_tx_msg_t;
extern UART_HandleTypeDef huart2;

extern volatile uint8_t motor2_align_request;
extern volatile uint8_t motor2_run_enable;
extern void motor2_fault_clear(void);          /* 定义在 main.c */

/* ISR RAM 抓取（定义在 main.c）：CAP arm 后，下一次 ID/IQ 给定变化开始记录 */
extern volatile uint8_t  cap_armed;
extern volatile uint8_t  cap_filling;
extern volatile uint16_t cap_n;

/* 速度环（定义在 main.c） */
extern foc_pi_t motor2_pi_s;
extern volatile float   motor2_speed_ref;
extern volatile uint8_t motor2_speed_enable;
extern volatile float   motor2_mech_speed;
extern volatile uint32_t motor2_angle_extrap_clip;
extern volatile uint8_t  caps_armed;
extern volatile uint16_t caps_n;
extern volatile float   motor2_speed_ff;

/* 位置环（定义在 main.c） */
extern foc_pi_t motor2_pi_pos;
extern volatile float   motor2_pos_mech;
extern volatile float   motor2_pos_ref;
extern volatile float   motor2_pos_err;
extern volatile uint8_t motor2_pos_enable;
extern volatile uint8_t motor2_pos_valid;
extern volatile uint32_t motor2_pos_gap_cnt;
extern int motor2_state_get(void);          /* 定义在 main.c：状态机当前状态号 */


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

#define CLI_TX_BUF_SIZE 256
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
        case CLI_TX_CLEAR_OK:
            strcpy(buf, "CLEAR OK\r\n");
            break;
        case CLI_TX_CAP_OK:
            strcpy(buf, "CAP ARMED\r\n");
            break;
        case CLI_TX_SPD_OK:
            strcpy(buf, "SPD OK (speed loop ON)\r\n");
            break;
        case CLI_TX_SPD_OFF_OK:
            strcpy(buf, "SPD OFF (speed loop OFF, iq_ref=0)\r\n");
            break;
        case CLI_TX_CAPS_OK:
            strcpy(buf, "CAPS ARMED (200Hz x 400pts = 2s)\r\n");
            break;
        case CLI_TX_POS_OK:
            strcpy(buf, "POS OK (position loop ON)\r\n");
            break;
        case CLI_TX_HELP:
            strcpy(buf, "CMD:\r\n"
                        "ALIGN\r\n"
                        "RUN\r\n"
                        "STOP\r\n"
                        "ID x\r\n"
                        "IQ x            (手动给定，会自动退出速度环)\r\n"
                        "SPD x           (速度环给定, rad/s 机械)\r\n"
                        "SPD_OFF         (关速度环, iq_ref=0)\r\n"
                        "FF x            (摩擦力前馈 A, 默认0.02; 低速爬行时调这个)\r\n"
                        "POS x           (位置环: 相对当前位置转 x 弧度, x>0 与 SPD 正方向一致)\r\n"
                        "POS_OFF         (关位置环)\r\n"
                        "PID_P kp ki     (位置环参数, 单位 1/s 和 1/s^2)\r\n"
                        "PID_D kp ki\r\n"
                        "PID_Q kp ki\r\n"
                        "PID_S kp ki     (单位 A/(rad/s), A/(rad/s)/s)\r\n"
                        "STATUS\r\n"
                        "CLEAR\r\n"
                        "CAP             (电流环 50us x 400 = 20ms)\r\n"
                        "CAPS            (速度环 5ms x 400 = 2s)\r\n"
                        "VOFA ON\r\n"
                        "VOFA OFF\r\n"
                        "HELP\r\n");
            break;
        case CLI_TX_STATUS:
            snprintf(buf, sizeof(buf),
                     "STATE %d\r\n"
                     "ANGLE %.3f\r\n"
                     "ID %.3f\r\n"
                     "IQ %.3f\r\n"
                     "ID_REF %.3f\r\n"
                     "IQ_REF %.3f\r\n"
                     "SPD %.3f\r\n"
                     "SPD_REF %.3f\r\n"
                     "SPD_EN %u\r\n"
                     "FF %.3f\r\n"
                     "POS %.3f\r\n"
                     "POS_REF %.3f\r\n"
                     "POS_ERR %.3f\r\n"
                     "POS_EN %u\r\n"
                     "POS_VALID %u\r\n"
                     "EXTRACLIP %lu\r\n",
                     motor2_state_get(),
                     motor2_electrical_angle_rad,
                     motor2_id, motor2_iq,
                     motor2_id_ref, motor2_iq_ref,
                     motor2_mech_speed, motor2_speed_ref,
                     (unsigned)motor2_speed_enable,
                     motor2_speed_ff,
                     motor2_pos_mech, motor2_pos_ref, motor2_pos_err,
                     (unsigned)motor2_pos_enable, (unsigned)motor2_pos_valid,
                     (unsigned long)motor2_angle_extrap_clip);
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
        if(cap_armed){ cap_armed=0; cap_n=0; cap_filling=1; }   /* CAP 后第一次给定变化即开始记录 */
        cli_enqueue(CLI_TX_REF_OK);
    }
    else if(sscanf(cmd,"IQ %f",&v1)==1)
    {
        motor2_speed_enable = 0;        /* 手动给 iq 自动退出速度环/位置环（人工优先） */
        motor2_pos_enable   = 0;
        motor2_iq_ref=v1;
        if(cap_armed){ cap_armed=0; cap_n=0; cap_filling=1; }
        cli_enqueue(CLI_TX_REF_OK);
    }
    else if(sscanf(cmd,"SPD %f",&v1)==1){
        motor2_pos_enable   = 0;                /* 手动给速度自动退出位置环 */
        motor2_speed_ref    = v1;               /* rad/s 机械角速度 */
        motor2_speed_enable = 1;
        foc_pi_reset(&motor2_pi_s);             /* 进环先清积分，避免突变 */
        cli_enqueue(CLI_TX_SPD_OK);
    }
    else if(strcmp(cmd,"SPD_OFF")==0){
        motor2_speed_enable = 0;
        motor2_iq_ref       = 0.0f;
        foc_pi_reset(&motor2_pi_s);
        cli_enqueue(CLI_TX_SPD_OFF_OK);
    }
    else if(sscanf(cmd,"PID_S %f %f",&v1,&v2)==2){
        motor2_pi_s.kp=v1;
        motor2_pi_s.ki=v2;
        cli_enqueue (CLI_TX_PID_OK);
    }
    else if(sscanf(cmd,"FF %f",&v1)==1){
        motor2_speed_ff = v1;        /* 摩擦力前馈(A)，0 = 关 */
        cli_enqueue(CLI_TX_REF_OK);
    }
    else if(sscanf(cmd,"POS %f",&v1)==1){
        /* 相对当前位置再转 v1 弧度（正方向与 SPD 正方向一致）。
         * 因为是"相对"，即使之前累圈因间隔过大而失效，这里也能重新锚定 -> 重新置 valid。 */
        motor2_pos_ref    = motor2_pos_mech + v1;
        motor2_pos_enable = 1;
        motor2_pos_valid  = 1;
        motor2_speed_enable = 1;                 /* 位置环要工作，内环必须在环 */
        foc_pi_reset(&motor2_pi_pos);
        foc_pi_reset(&motor2_pi_s);
        cli_enqueue(CLI_TX_POS_OK);
    }
    else if(strcmp(cmd,"POS_OFF")==0){
        motor2_pos_enable   = 0;
        motor2_speed_enable = 0;
        motor2_iq_ref       = 0.0f;
        foc_pi_reset(&motor2_pi_pos);
        cli_enqueue(CLI_TX_SPD_OFF_OK);
    }
    else if(sscanf(cmd,"PID_P %f %f",&v1,&v2)==2){
        motor2_pi_pos.kp=v1;
        motor2_pi_pos.ki=v2;
        cli_enqueue (CLI_TX_PID_OK);
    }
    else if(strcmp(cmd,"CAPS")==0){
        caps_armed = 1;
        caps_n     = 0;
        cli_enqueue(CLI_TX_CAPS_OK);
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
    else if(strcmp(cmd,"CLEAR")==0){
        motor2_fault_clear();
        cli_enqueue (CLI_TX_CLEAR_OK);

    }
    else if(strcmp(cmd,"CAP")==0){
        cap_armed = 1;
        cli_enqueue (CLI_TX_CAP_OK);

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
