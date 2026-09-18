#ifndef MOTOR_CLI_H
#define MOTOR_CLI_H

#include "main.h"
#include <stdint.h>

void motor_cli_poll(void);
void motor_cli_init(void);
void motor_cli_rx_char(uint8_t ch);

#endif
