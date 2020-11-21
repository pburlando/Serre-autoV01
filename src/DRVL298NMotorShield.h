/**
 * 	@file 	DRVL298NMotorShield.h
 *	@brief 	Pilotage de deux moteurs à courant continu,
 *			deux sens de marche,
 *			contrôle des vitesses en MLI
 * 	@date 	28 avr. 2019
 * 	@author patrice
 * 	@version 0.1
 */

#ifndef DRVL298NMOTORSHIELD_H_
#define DRVL298NMOTORSHIELD_H_

#include "Arduino.h"

class DRVL298NMotorShield {
private:
	const uint8_t _mgDir; // pin de contrôle de la direction pour le moteur gauche IN3
	const uint8_t _mgPwm; // pin de contrôle de la vitess pour le moteur gauche IN4
	const uint8_t _mdDir; // pin de contrôle de la direction pour le moteur droit IN2
	const uint8_t _mdPwm; // pin de contrôle de la vitess pour le moteur droit IN1
	const bool _mgFlip;   // bit pour inverser le sens de rotation du moteur gauche
	const bool _mdFlip;   // bit pour inverser le sens de rotation du moteur droit
public:
	DRVL298NMotorShield(uint8_t mgDir, uint8_t mgPwm, uint8_t mdDir, uint8_t mdPwm, bool mgFlip, bool mdFlip);
	virtual ~DRVL298NMotorShield();
	void setSpeeds(int speedLeft, int speedRight) const;  // Vitesse comprise entre -255 et 255, positif = marche avant
	void setSpeed_left(int speed) const;
	void setSpeed_right(int speed) const;
};

#endif /* DRVL298NMOTORSHIELD_H_ */
