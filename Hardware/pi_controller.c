
#include "pi_controller.h"




void foc_pi_init(foc_pi_t* pi,float kp,float ki,float output_min,float output_max){
    
    pi->kp=kp;
    pi->ki=ki;
    pi->integral=0.0f;
    pi->output_min=output_min;
    pi->output_max=output_max;

}

void foc_pi_reset(foc_pi_t* pi){

    pi->integral=0.0f;
}

float foc_pi_update(foc_pi_t* pi,float error,float dt){
    
    float proportional;
    float new_integral;
    float output;
    proportional = pi->kp*error;
    new_integral = pi->integral + pi->ki*error*dt;
    output=proportional+new_integral;
    if(output > pi->output_max){
        output = pi->output_max;
        if (error < 0.0f)
        {
            pi->integral = new_integral;
        }
    }
    else if(output < pi->output_min){
        output = pi->output_min;
        if (error > 0.0f)
        {
            pi->integral = new_integral;
        }
    }
    else
    {
        pi->integral = new_integral;
    }
    return output;
}

