#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "platform.h"

#include "define.h"

#include "actions.h"

// Fonction utilisée pour tester la présence de la tirette
// Retourne 1 si la tierre est encore la
// 0 Si elle est absente
uint8_t presence_tirette(void)
{
 return (!platform_gpio_get_value(GPIO_TIRETTE));
}

//Fonction utilisé pour tester la couleur du camp de départ
//Retourne COULEUR_JAUNE si la couleur est Jaune // A verifié
//Retourne COULEUR_ROUGE si la couleur est Rouge // A vérifié
uint8_t couleur_depart()
{
  return (platform_gpio_get_value(GPIO_SELECTION_COULEUR));
}

//Fonction utilisée pour tester si le capteur à contact en bout de canon est actif ou non
//retourne 1 si le capteur detecte quelque chose
//retourne 0 sinon
uint8_t contact_fresque()
{
  return (platform_gpio_get_value(GPIO_CONTACT_CANON));
}

//Fonction utilisée pour activer la turbine
void enable_turbine()
{
  platform_gpio_reset(GPIO_ENABLE_TURBINE);
}

//Fonction utilisée pour desactiver la turbine
void disable_turbine()
{
  platform_gpio_set(GPIO_ENABLE_TURBINE);
}

//Fonction utilisée pour désactiver la puissance (relais)
void enable_power_relay(void)
{
  platform_gpio_reset(GPIO_RELAIS);
}

//Fonction utilisée pour activer la puissance (relais)
void disable_power_relay(void)
{
  platform_gpio_set(GPIO_RELAIS);
}

//Fonction utilisée pour bouger un servo connecté sur servo_module
//servo peut-etre un des 8 servos définis dans define.h
void move_servo_from_servo_module(uint8_t servo, uint8_t angle)
{
  angle = (angle > 100) ? 100 : angle;
  //printf("angle :%d \r\n", angle);
  CanTxMsg CAN_Tx;
  uint8_t mailbox_number = 0;
  if(servo & SERVO_FROM_MODULE_0)
  {
    CAN_Tx.StdId = 0x80;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_1)
  {
    CAN_Tx.StdId = 0x81;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_2)
  {
    CAN_Tx.StdId = 0x82;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_3)
  {
    CAN_Tx.StdId = 0x83;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_4)
  {
    CAN_Tx.StdId = 0x84;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_5)
  {
    CAN_Tx.StdId = 0x85;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_6)
  {
    CAN_Tx.StdId = 0x86;
    CAN_Tx.Data[0] = angle;
  }

  if(servo & SERVO_FROM_MODULE_7)
  {
    CAN_Tx.StdId = 0x87;
    CAN_Tx.Data[0] = angle;
  }

  CAN_Tx.ExtId = 0;
  CAN_Tx.IDE = CAN_Id_Standard;
  CAN_Tx.RTR = CAN_RTR_Data;
  CAN_Tx.DLC = 1;

  mailbox_number = CAN_Transmit(CAN1, &CAN_Tx);
  if(mailbox_number == CAN_TxStatus_NoMailBox)
    platform_led_set(PLATFORM_LED3);
  uint8_t transmit_status = CAN_TransmitStatus(CAN1, mailbox_number);
  while(transmit_status!=CAN_TxStatus_Ok){
    transmit_status = CAN_TransmitStatus(CAN1, mailbox_number);
    if(transmit_status == CAN_TxStatus_Ok)
    {
      platform_led_set(PLATFORM_LED4);
      platform_led_reset(PLATFORM_LED7);
    }
    else if( transmit_status == CAN_TxStatus_Pending)
    {
      platform_led_set(PLATFORM_LED7);
    }
    else
      platform_led_set(PLATFORM_LED6);
  }

  //vTaskDelay(100);
}

//Fonctions utilitaire pour bouger les servos a des positions fixes
void ouvrir_bras_gauche()
{
  move_servo_from_servo_module(SERVO_BRAS_GAUCHE,SERVO_BRAS_GAUCHE_POSITION_OUVERTE);
}

void fermer_bras_gauche()
{
  move_servo_from_servo_module(SERVO_BRAS_GAUCHE,SERVO_BRAS_GAUCHE_POSITION_FERMEE);
}

void ouvrir_bras_droit()
{
  move_servo_from_servo_module(SERVO_BRAS_DROITE,SERVO_BRAS_DROIT_POSITION_OUVERTE);
}

void fermer_bras_droit()
{
  move_servo_from_servo_module(SERVO_BRAS_DROITE,SERVO_BRAS_DROIT_POSITION_FERMEE);
}

void init_servo_peinture_ausbee()
{
  move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE, SERVO_PEINTURE_COTE_AUSBEE_POSITION_FERMEE);
}

void placer_peinture_ausbee()
{
  move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE,SERVO_PEINTURE_COTE_AUSBEE_POSITION_OUVERTE);
}

void init_servo_peinture_canon()
{
  move_servo_from_servo_module(SERVO_PEINTURE_CANON, SERVO_PEINTURE_CANON_POSITION_FERMEE);
}

void placer_peinture_canon()
{
  move_servo_from_servo_module(SERVO_PEINTURE_CANON, SERVO_PEINTURE_CANON_POSITION_OUVERTE);
}

void ouvrir_servo_canon_haut()
{
  move_servo_from_servo_module(SERVO_CANON_HAUT,SERVO_CANON_HAUT_POSITION_OUVERTE);
}

void fermer_servo_canon_haut()
{
  move_servo_from_servo_module(SERVO_CANON_HAUT,SERVO_CANON_HAUT_POSITION_FERMEE);
}

void ouvrir_servo_canon_bas()
{
  move_servo_from_servo_module(SERVO_CANON_BAS, SERVO_CANON_BAS_POSITION_OUVERTE);
}

void fermer_servo_canon_bas()
{
  move_servo_from_servo_module(SERVO_CANON_BAS, SERVO_CANON_BAS_POSITION_FERMEE);
}

//Lance une balle
//Necessite d'activer la turbine 10 secondes avant
//séquence: ouvre servo bas
//          ferme servo bas
//          ouvre servo haut
//          ferme servo haut
void lancer_une_balle()
{
  ouvrir_servo_canon_bas();
  vTaskDelay(60);
  fermer_servo_canon_bas();
  vTaskDelay(60);
  ouvrir_servo_canon_haut();
  vTaskDelay(60);
  fermer_servo_canon_haut();
  vTaskDelay(60);
}


