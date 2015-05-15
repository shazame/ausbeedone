#ifndef INIT_H
#define INIT_H

#include "misc.h"
#include "platform.h"
#include <AUSBEE/servo.h>
#include <AUSBEE/l298_driver.h>
#include <AUSBEE/lidar.h>

void init_can();
void init_can_rx_interrupt();
void init_usart_interrupt();
//void init_servo();
void init_mot(struct ausbee_l298_chip* mot_droit, struct ausbee_l298_chip *mot_gauche);
void init_encoders();
void init_turbine();
void init_lidar();
void init_gpio_robot();
void init_timer_relais();
#endif //INIT_H
