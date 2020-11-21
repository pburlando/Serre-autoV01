/**
 * 	@file 	main.cpp
 *	@brief 	Programme de la serre automatisée 
 * 	@date 	23 juillet 2020
 * 	@author patrice Lycée Raynouard
 * 	@version 0.1
 *
 */

#include <Arduino.h>
#include <SerreAuto.h>
#include "WatchDog.h"

SerreAuto serre;
unsigned long start_time = millis();
void atuISR();

void setup() {
  /**
   * @brief Initialisation au boot
   * 
   */
  serre.init();
  serre.afficher_version();
  serre.ouvrir_volet();
  serre.fermer_volet();
  serre.commander_eclairage(true);
  serre.commander_extracteur_air(255);
  delay(2000); 
  WatchDog::init(atuISR, 10000);
  
}

/**
 * @brief boucle principale
 * 
 */
void loop() {

  static uint8_t compteur_de_boucle;
  WatchDog::start();
  // Code à répéter toutes les secondes
  if((millis() - start_time) > 999) {
    start_time = millis();
    float ta = serre.temperature_ambiante();
    serre.humidite_ambiante();
    serre.temperature_plateau();
    
    // Gestion de l'aération et du chauffage
    if(ta >= CONSIGNE_TA + HYSTERESIS_TA) {
      serre.ouvrir_volet();
      serre.commander_extracteur_air(255);
      if (serre.chauffer(false) != 0) {
        serre.afficher_erreur(0);
      }
    }
    else if (serre.temperature_ambiante() <= (CONSIGNE_TA - HYSTERESIS_TA)) {
      serre.fermer_volet();
      serre.commander_extracteur_air(60);
      if (serre.chauffer(true) != 0) {
        serre.afficher_erreur(0);
      }
    }

    // Modification de l'affichage toutes les 10 secondes
    if(compteur_de_boucle < 3)
      serre.afficher_version();
      else if(compteur_de_boucle < 25)
        serre.afficher_ambiance();
    else compteur_de_boucle = 0;
    compteur_de_boucle += 1;

    // Gestion de l'éclairage selon l'horloge
    // heure de début (0-23), durée d'éclairement (1 - 23) 0 toujours éclairée
    serre.commander_eclairage(serre.autoriser_eclairage(6,14));

    // Gestion de l'arrosage
    // heure de début, durée en seconde, périodicité 
    serre.arroser(serre.autoriser_arrosage(10, 15, 1));

    // Superviser les paramètres sur le moniteur série
    serre.superviser();
  }
  WatchDog::stop();
}

/**
 * @brief routine d'interruption de l'arrêt d'urgence 
 *        levée par le timer chien de garde
 *        pour prévenir un plantage du programme principal
 *        et couper l'alimentation du plateau chauffant
 * 
 */
void atuISR() {
  pinMode(8, OUTPUT);
  digitalWrite(8,0);
  pinMode(10, OUTPUT);
  digitalWrite(8,0);
  digitalWrite(9, 0);
  Serial.println("ATU");
  serre.chauffer(false);
  serre.arroser(false);
  serre.commander_extracteur_air(0);
  serre.commander_eclairage(false);
  serre.afficher_arret_urgence();
  serre.fermer_volet();
  while(true);
}