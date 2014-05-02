#ifndef ACTIONS_H
#define ACTIONS_H
#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "define.h"


uint8_t presence_tirette();
uint8_t couleur_depart();
uint8_t contact_fresque();

void enable_turbine();
void disable_turbine();

void move_servo_from_servo_module(uint8_t servo, uint8_t angle);

void init_servo_position_depart();

void ouvrir_bras_gauche();
void fermer_bras_gauche();

void ouvrir_bras_droit();
void fermer_bras_droit();

void init_servo_peinture_ausbee();
void placer_peinture_ausbee();

void init_servo_peinture_canon();
void placer_peinture_canon();

void ouvrir_servo_canon_haut();
void fermer_servo_canon_haut();

void ouvrir_servo_canon_bas();
void fermer_servo_canon_bas();

void lancer_une_balle();

#endif //ACTIONS_H
