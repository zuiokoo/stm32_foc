#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H



typedef struct{

    float  kp;
    float  ki;
    float  integral;
    
    float   output_min;
    float   output_max;

}foc_pi_t;




void foc_pi_init(foc_pi_t* pi,float kp,float ki,float output_min,float output_max);

void foc_pi_reset(foc_pi_t* pi);

float foc_pi_update(foc_pi_t* pi,float error,float dt);











#endif
