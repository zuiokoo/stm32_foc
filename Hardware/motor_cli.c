#include "motor_cli.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "pi_controller.h"

extern UART_HandleTypeDef huart2;

extern volatile uint8_t motor1_align_request;
extern volatile uint8_t motor1_run_enable;


extern float motor1_id_ref;
extern float motor1_iq_ref;


extern foc_pi_t motor1_pi_d;
extern foc_pi_t motor1_pi_q;


extern float motor1_id;
extern float motor1_iq;

extern float motor1_electrical_angle_rad;

extern uint8_t uart2_rx_byte;

#define CLI_BUF_SIZE 64
static char cli_buf[CLI_BUF_SIZE];
static uint8_t cli_index=0;

void motor_cli_init(void){



}
static void cli_send(char* str){
    HAL_UART_Transmit( &huart2,(uint8_t*)str, strlen(str),100);
}
static void cli_status(void)
{

    char buf[128];

    sprintf(buf,"STATE\r\n""ANGLE %.3f\r\n""ID %.3f\r\n""IQ %.3f\r\n""ID_REF %.3f\r\n""IQ_REF %.3f\r\n",motor1_electrical_angle_rad,motor1_id,motor1_iq,motor1_id_ref,motor1_iq_ref);

    cli_send(buf);

}
static void cli_help(void)
{
    cli_send("CMD:\r\n""ALIGN\r\n""RUN\r\n""STOP\r\n""ID x\r\n""IQ x\r\n""PID_D kp ki\r\n""PID_Q kp ki\r\n""STATUS\r\n");
}
void motor_cli_parse(char*cmd){
    float v1,v2;
    if(strcmp(cmd,"ALIGN")==0){
        motor1_align_request=1;
        cli_send("ALIGN OK\r\n");
    }
    else if(strcmp(cmd,"RUN")==0){
        motor1_run_enable=1;
        cli_send("RUN OK\r\n");
    }
    else if(strcmp(cmd,"STOP")==0){
        motor1_run_enable=0;
        cli_send("STOP OK\r\n");  
    }
    else if(sscanf(cmd,"ID %f",&v1)==1){
        motor1_id_ref=v1;
    }
    else if(sscanf(cmd,"IQ %f",&v1)==1)
    {
        motor1_iq_ref=v1;
    }
    else if(sscanf(cmd,"PID_D %f %f",&v1,&v2)==2){
        motor1_pi_d.kp=v1;
        motor1_pi_d.ki=v2;
    }
    else if(sscanf(cmd,"PID_Q %f %f",&v1,&v2)==2){
        motor1_pi_q.kp=v1;
        motor1_pi_q.ki=v2;
    } 
    else if(strcmp(cmd,"STATUS")==0){
        cli_status();
    }
    else if(strcmp(cmd,"HELP")==0)
    {
        cli_help();
    }
    else
    {
        cli_send("ERR\r\n");
    }
            
}
void motor_cli_rx_char(uint8_t ch){

    if(ch=='\r'||ch=='\n'){
        if(cli_index){
            cli_buf[cli_index]=0;
            motor_cli_parse(cli_buf);
            cli_index=0;
        }
    
    
    }
    else{
    
        if(cli_index<CLI_BUF_SIZE-1){
        
            cli_buf[cli_index++]=ch;
        }
    
    
    }


}
