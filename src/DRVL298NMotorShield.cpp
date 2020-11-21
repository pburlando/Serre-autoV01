/**
 * 	@file 	DRVL298NMotorShield.cpp
 *	@brief 	Pilotage de deux moteurs à courant continu,
 *			deux sens de marche,
 *			contrôle des vitesses en MLI
 * 	@date 	28 avr. 2019
 * 	@author patrice
 * 	@version 0.1
 */

#include "DRVL298NMotorShield.h"
#include "Arduino.h"

DRVL298NMotorShield::DRVL298NMotorShield(uint8_t mgDir, uint8_t mgPwm, uint8_t mdDir, uint8_t mdPwm, bool mgFlip, bool mdFlip)
	: _mgDir(mgDir), _mgPwm(mgPwm), _mdDir(mdDir), _mdPwm(mdPwm), _mgFlip(mgFlip), _mdFlip(mdFlip) {
		/**
		 * @brief Construct a new pin Mode object
		 * 
		 */
		pinMode(_mgDir, OUTPUT);
		pinMode(_mgPwm, OUTPUT);
		pinMode(_mdDir, OUTPUT);
		pinMode(_mdPwm, OUTPUT);
	}


DRVL298NMotorShield::~DRVL298NMotorShield() {
	// TODO Auto-generated destructor stub
}


void DRVL298NMotorShield::setSpeeds(int speedLeft, int speedRight) const {
	/**
	 * 	@brief Fixer les vitesses des moteurs droit et gauche
	 * 	@param speedLeft Vitesse moteur gauche bornée à 255 positif avant négatif arrière
	 *	@param speedRight Vitesse moteur droit bornée à 255 positif avant négatif arrière
	 */

	setSpeed_left(speedLeft);
	setSpeed_right(speedRight);

}


void DRVL298NMotorShield::setSpeed_left(int speed) const {
	/**
	 * 	@brief Fixer la vitesse du moteur gauche (OUT2)
	 * 	@param speed Vitesse moteur gauche bornée à 255 positif avant négatif arrière
	 */

	bool dirLeft = 0;

	// Changement du signe des vitesses selon l'attribut Flip
	if(_mgFlip)
		speed = -speed;


	// Traitement de la vitesse du moteur gauche
	if(speed < 0) {
		speed = -speed;
		dirLeft = 1;
	}
	if(speed > 255)
		speed = 255;

	// Ecriture du sens de rotation
	digitalWrite(_mgDir, dirLeft);

	// Ecriture de la vitesse moteur gauche
	if(dirLeft)
		analogWrite(_mgPwm, 255 - speed);
	else
		analogWrite(_mgPwm, speed);

}


void DRVL298NMotorShield::setSpeed_right(int speed) const {
	/**
	 * 	@brief Fixer la vitesse du moteur droit (out3)
	 *	@param speed Vitesse moteur droit bornée à 255 positif avant négatif arrière
	 */
	bool dirRight = 0;

	// Changement du signe de la vitesse selon l'attribut Flip
	if(_mdFlip)
		speed = -speed;

	// Traitement de la vitesse du moteur droit
	if(speed < 0) {
		speed = -speed;
		dirRight = 1;
	}
	if(speed > 255)
		speed = 255;

	// Ecriture du sens de rotation droit
	digitalWrite(_mdDir, dirRight);

	// Ecriture de la vitesse moteur droit
	if(dirRight)
		analogWrite(_mdPwm, 255 - speed);
	else
		analogWrite(_mdPwm, speed);

}


