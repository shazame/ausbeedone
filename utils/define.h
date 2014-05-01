#ifndef UTILS_DEFINE_H
#define UTILS_DEFINE_H


// Servomoteur sur servo module
#define SERVO_FROM_MODULE_0 0x01  //canon bas
#define SERVO_FROM_MODULE_1 0x02  //canon haut
#define SERVO_FROM_MODULE_2 0x04  //Bras gauche (coté ausbee) rentré :12 ouvert: 70
#define SERVO_FROM_MODULE_3 0x08  //Bras droit (alim) rentré: 91 ouvert : 35
#define SERVO_FROM_MODULE_4 0x10  //peinture ausbee sortie: 4 rentré: 52
#define SERVO_FROM_MODULE_5 0x20  //peinture canon sortie: 80 rentré : 40
#define SERVO_FROM_MODULE_6 0x40  // not used
#define SERVO_FROM_MODULE_7 0x80  // not used

#define SERVO_CANON_BAS            SERVO_FROM_MODULE_0 //fermé :79  ouvert: 50
#define SERVO_CANON_HAUT           SERVO_FROM_MODULE_1 //fermé :24  ouvert: 62
#define SERVO_BRAS_GAUCHE          SERVO_FROM_MODULE_2
#define SERVO_BRAS_DROITE          SERVO_FROM_MODULE_3
#define SERVO_PEINTURE_COTE_AUSBEE SERVO_FROM_MODULE_4
#define SERVO_PEINTURE_CANON       SERVO_FROM_MODULE_5

#define SERVO_BRAS_GAUCHE_POSITION_FERMEE           12
#define SERVO_BRAS_GAUCHE_POSITION_OUVERTE          70

#define SERVO_BRAS_DROIT_POSITION_FERMEE            92
#define SERVO_BRAS_DROIT_POSITION_OUVERTE           35    

#define SERVO_CANON_BAS_POSITION_FERMEE             79
#define SERVO_CANON_BAS_POSITION_OUVERTE            50

#define SERVO_CANON_HAUT_POSITION_FERMEE            24
#define SERVO_CANON_HAUT_POSITION_OUVERTE           62

#define SERVO_PEINTURE_COTE_AUSBEE_POSITION_FERMEE  52
#define SERVO_PEINTURE_COTE_AUSBEE_POSITION_OUVERTE 4

#define SERVO_PEINTURE_CANON_POSITION_FERMEE        40
#define SERVO_PEINTURE_CANON_POSITION_OUVERTE       80

// LIDAR
// Detection de zone circulaire
#define DISTANCE_CENTER_TO_LIDAR  105.0 //Distance du Lidar au centre du cercle de detection
#define CIRCLE_LIDAR_DIAMETER     360.0 //Diametre de la zone de detection circulaire

//GPIO

#define GPIO_ENABLE_TURBINE     GPIO1
#define GPIO_RELAIS             GPIO2
//#define GPIO3
//#define GPIO4
//#define GPIO5
#define GPIO_SELECTION_COULEUR  GPIO6
//#define GPIO7 
#define GPIO_TIRETTE            GPIO8
#define GPIO_CONTACT_CANON      GPIO9

//Couleur de départ
#define COULEUR_JAUNE 0     // A VERIFIER
#define COULEUR_ROUGE 1     // A VERIFIER

#endif //UTILS_DEFINE_H
