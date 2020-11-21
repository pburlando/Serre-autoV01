/**
 * 	@file 	SerreAuto.h
 *	@brief 	Contrôle de la serre automatisée
 * 	@date 	23 juillet 2020
 * 	@author patrice
 * 	@version 0.1
 */

#ifndef SERREAUTO_H_
#define SERREAUTO_H_

#include "Arduino.h"
#include <RTClib.h>

#define VERSION "V0.1 "
#define CONSIGNE_TA 20
#define HYSTERESIS_TA 1
#define CONSIGNE_TC 60

class SerreAuto {
   
private:
    float _temperature_plateau;         /** Température du plateau chauffant en degré Celsius */
    float _temperature_ambiante;        /** Température ambiante à l'intérieur de la serre en degré Celsius */
    float _humidite_ambiante;           /** Humidité ambiante relative à l'intérieur de la serre en % */
    uint8_t _temperature_consigne = 60; /** Température de consigne du plateau chauffant en degré Celsius */
    float _commande_mli;                /** Valeur de la commande MLI du plateau chauffant 0 à 255 */
    uint8_t _jour_arrosage_valide;      /** Jour de la semaine où l'arrosage est valide. jour 0 = dimanche */
    uint8_t _heure_debut_arrosage;      /** Heure de début de l'arrosage 0-23 */
    uint16_t _duree_arrosage;            /** Durée d'arrosage en seconde. Approximativement 10cl pour 13s */
    uint8_t _periodicite_arrosage;
    bool _etat_eclairage;               /** Etat de l'éclairage de la serre 0 = éteint */
    bool _etat_arrosage;                /** Etat de la pompe d'arrosage 1 = pompe en marche */
    bool _etat_volet;                   /** Etat du volet de la serre. 1 = volet ouvert */
    uint8_t _mli_extracteur;            /** Valeur de la commande MLI de l'extracteur d'air de 0 à 255 */
    DateTime _now;                      /** Date du jour */

public:
    SerreAuto();
    virtual ~SerreAuto();
    void init();
    void commander_extracteur_air(int vitesse);
    void commander_eclairage(bool etat);
    int ouvrir_volet();
    int fermer_volet();
    int chauffer(bool etat);
    float temperature_plateau();
    float humidite_ambiante();
    float temperature_ambiante();
    void afficher_ambiance() const;
    void superviser() const;
    void arroser(bool);
    void afficher_version() const;
    void afficher_arret_urgence() const;
    bool autoriser_eclairage(uint8_t debut, uint8_t duree);
    bool autoriser_arrosage(uint8_t debut, uint16_t duree, uint8_t periodicite);
    void jour_arrosage_valide(uint8_t);
    uint8_t jour_arrosage_valide() const;
    uint8_t mli_plateau_chauffant() const;
    void enregistrer_parametres_arrosage();
    void restaurer_parametres_arrosage();
    void afficher_erreur(uint8_t err_number);

};

#endif /* SERREAUTO_H_ */