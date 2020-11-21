/**
 * 	@file 	SerreAuto.cpp
 *	@brief 	Contrôle de la serre automatisée
 * 	@date 	23 juillet 2020
 * 	@author patrice
 * 	@version 0.1
 * 
 */

#include "SerreAuto.h"
#include "DRVL298NMotorShield.h"
#include "Arduino.h"
#include "thermistortable.h"
#include <PID_v1.h>
#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"


// Brochage sur carte ramps 1.4
#define CDEVOLETDIR 16
#define CDEVOLETPWM 17
#define EXTRACTEURDIR 23
#define EXTRACTEURPWM 25
#define FDCVOLETOUVERT 2    // Actif à l'état bas
#define FDCVOLETFERME 3     // Actif à l'état bas
#define DHTPIN 40
#define DHTTYPE DHT11
#define THERM0 A13
#define HEAT_BED 8
#define ECLAIRAGE 9
#define POMPE 10


// Variables globales pour régulation de température du plateau
double Input, Setpoint, Output;
double Kp = 62.5;
double Ki = 0.2083;
double Kd = 31.25;


DRVL298NMotorShield drv(CDEVOLETDIR, CDEVOLETPWM, EXTRACTEURDIR, EXTRACTEURPWM, true, false);

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

DHT dht(DHTPIN, DHTTYPE);

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

RTC_DS1307 rtc;


/**
 * @brief Construct a new pin Mode object
 * 
 */
SerreAuto::SerreAuto() {

    pinMode(ECLAIRAGE, OUTPUT);
    pinMode(POMPE, OUTPUT);
    pinMode(FDCVOLETFERME, INPUT);
    pinMode(FDCVOLETOUVERT, INPUT);        
	}


/**
 * @brief Initialisation des capteurs i2c, 
 *        de l'afficheur LCD,
 *        de l'horloge rtc,
 *        de la communication série,
 *        du régulateur PID et
 *        du jour d'arrosage valide
 * 
 */
void SerreAuto::init() {
    Serial.begin(9600);
    Serial.print("Serre automatise ");
    Serial.println(VERSION);
    dht.begin();
    Serial.println("dht OK");
    lcd.init();
    lcd.backlight();
    Serial.println("lcd OK");

    // Réglage du correcteur PID pour réguler la température du plateau chauffant

    Setpoint = _temperature_consigne;  // Température de consigne en degré celsius
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);  // Commande MLI de la plaque chauffante
    myPID.SetSampleTime(1500);  // Temps d'échantillonnage en ms
    Serial.println("PID ok");

    // Connexion avec l'horloge RTC
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        abort();
    } 

    if (! rtc.isrunning()) {
        Serial.println("RTC is NOT running, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    // When time needs to be re-set on a previously configured device, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Initialiser le jour d'arrosage valide au démarrage de 0 à 6. La semaine commence dimanche.
    DateTime now = rtc.now();
    jour_arrosage_valide(now.dayOfTheWeek());
    Serial.println("RTC ok");
}


/**
 * @brief Commander le ventilateur d'extraction d'air
 * @param vitesse à la vitesse 0 < vitesse < 255
 * 
 */
void SerreAuto::commander_extracteur_air(int vitesse) {
    drv.setSpeed_right(vitesse);
    _mli_extracteur = vitesse;
}

/**
 * @brief Commander l'éclairage de la serre
 * @param etat état binaire de l'éclairage. 0 = éteint
 * 
 */
void SerreAuto::commander_eclairage(bool etat) {

    digitalWrite(ECLAIRAGE, etat);
    _etat_eclairage = etat;
}


/**
 * @brief Ouvrir le volet jusqu'au fin de course
 * @return 0 si pas d'erreur sinon -1
 * 
 */
int SerreAuto::ouvrir_volet() {
    static unsigned long old_time = millis();
    if(digitalRead(FDCVOLETOUVERT))
        drv.setSpeed_left(150);
    while(digitalRead(FDCVOLETOUVERT)) {
        if ((millis() - old_time) > 8000) {
            Serial.println("Erreur ouverture volet");
            return -1;
        }
    }
    drv.setSpeed_left(0);
    _etat_volet = true;
    //Serial.println("Volet Ouvert");
    return 0;

}


/**
 * @brief Fermer le volet jusqu'au fin de course
 * @return 0 si pas d'erreur sinon -1
 */
int SerreAuto::fermer_volet() {
    static unsigned long old_time = millis();
    if(digitalRead(FDCVOLETFERME))
        drv.setSpeed_left(-150);
    while(digitalRead(FDCVOLETFERME)) {
        if ((millis() - old_time) > 8000) {
            Serial.println("Erreur fermeture volet");
            return -1;
        }
    }
    drv.setSpeed_left(0);
    _etat_volet = false;
    //Serial.println("Volet Ferme");
    return 0;
}


/**
 * @brief Autoriser le chauffage de la serre. La température du plateau est régulée à 60°C
 * @param etat état binaire du chauffage.
 * @return -1 en cas d'erreur sinon 0
 * 
 */
int SerreAuto::chauffer(bool etat) {
    Input = temperature_plateau();
    if ((Input < 7) | (Input > 65)) {
        analogWrite(HEAT_BED, 0);
        _commande_mli = 0;
        return -1;

    }

    if(etat) {
        if (Input < 58) {
            analogWrite(HEAT_BED, 255);
            _commande_mli = 255;
            return 0;
        }
        else if (Input > 62) {
            analogWrite(HEAT_BED, 0);
            _commande_mli = 0;
            return 0;
        }
        myPID.Compute();  // Met à jour la variable globale Output selon la température du plateau chauffant
        analogWrite(HEAT_BED, Output);
        _commande_mli = Output;
        return 0;
    }
    else  {
        analogWrite(HEAT_BED, 0);  // Désactive le plateau chauffant si le chauffage n'est pas demandé
        _commande_mli = 0;
        return 0;
    }
    analogWrite(HEAT_BED, 0);
    _commande_mli = 0;
    return -1;
}


/**
 * @brief Acquérir la température du plateau chauffant en degré Celsius. Le capteur est une thermistance de 100k. La température est calculée par interpolation linéaire d'une table d'étalonnage.
 * @return la valeur de la température en degré celsius.
 */
float SerreAuto::temperature_plateau() {
    
   int len_temptable = sizeof(temptable)/sizeof(temptable[0]);
   float celsius = 0;
   int i = 0;
   int raw = analogRead(THERM0);

   while(raw > temptable[i][0]) {   // c
      // Parcourir la table pour trouver la valeur la plus proche par excés (immédiatement supérieure à raw)
      i++;
      if (i == len_temptable) break;  // Sortir de la boucle si on a parcouru tout le tableau
   }
   
   if (i == 0) {
      celsius = temptable[i][1];  // La température est supérieure à 300°C
   }
   
   else if (i == len_temptable) {
      // La température est inférieure à 0°C
      celsius = temptable[i - 1][1];
   }
 
   else {
      // La température est comprise entre la valeur actuelle de la table et la valeur précédente
      // On calcule la valeur de la température par interpolation linéaire
      celsius = temptable[i][1] + (temptable[i][0] - raw) * float((temptable[i-1][1] - temptable[i][1])) / float((temptable[i][0] - temptable[i-1][0]));
   }
   _temperature_plateau = celsius;
   return _temperature_plateau;
}


/**
 * @brief Mesure l'humidité ambiante de la serre. Capteur DHT11.
 * @return Le taux d'humidité relative dans l'air (%)
 * 
 */
float SerreAuto::humidite_ambiante() {

    _humidite_ambiante = dht.readHumidity();
    return _humidite_ambiante;
}


/**
 * @brief Mesure la température ambiante mesurée par la sonde dht11
 * @return La température ambiante en degré Celsius.
 * 
 */
float SerreAuto::temperature_ambiante() {

    _temperature_ambiante = dht.readTemperature();
    return _temperature_ambiante;
}


/**
 * @brief Ecrit les données mesurées sur le lcd
 * 
 */
void SerreAuto::afficher_ambiance() const{
    char data[20];
    char ha_str[5];
    char ta_str[5];
    char tp_str[5];

    dtostrf(_humidite_ambiante, 4, 1, ha_str);
    dtostrf(_temperature_ambiante, 4, 1, ta_str);
    dtostrf(_temperature_plateau, 4, 1, tp_str);    
    // Créer une chaine formatée
    sprintf(data, "Tp=%s C  Tc=%d C", tp_str, _temperature_consigne);
    lcd.setCursor(0,0);
    lcd.print(data);
    sprintf(data, "H=%s  T=%s C", ha_str, ta_str);
    lcd.setCursor(0,1);
    lcd.print(data);

}


/**
 * @brief Ecrit la version du firmware, la date et l'heure sur le lcd
 * 
 */
void SerreAuto::afficher_version() const{
    char data[16];
    char buf2[] = "DD/MM/YY hh:mm  ";
    lcd.setCursor(0,0);
    sprintf(data, "Serre auto %s", VERSION);
    lcd.println(data);
    DateTime now = rtc.now();
    lcd.setCursor(0,1);
    lcd.print(now.toString(buf2));
}


/**
 * @brief Envoie les grandeurs mesurées, l'état des capteurs et les commandes horodatés sur le port USB
 * 
 */
void SerreAuto::superviser() const{
    DateTime now = rtc.now();
    char buf1[] = "DDD DD MMM YYYY hh:mm:ss";
    Serial.print(now.toString(buf1));
    Serial.print(", ");
    Serial.print(_temperature_consigne);
    Serial.print(", ");
    Serial.print(_temperature_plateau);
    Serial.print(", ");
    Serial.print(_commande_mli);
    Serial.print(", ");    
    Serial.print(_temperature_ambiante);
    Serial.print(", ");
    Serial.print(_humidite_ambiante);
    Serial.print(", ");
    Serial.print(_etat_eclairage);
    Serial.print(", ");
    Serial.print(_etat_arrosage);
    Serial.print(", ");
    Serial.print(_jour_arrosage_valide);
    Serial.print(", ");
    Serial.print(_etat_volet);
    Serial.print(", ");
    Serial.println(_mli_extracteur);
}


/**
 * @brief Active la pompe d'arrosage
 * @param autorisation état binaire de la pompe d'arrosage.
 * 
 */
void SerreAuto::arroser(bool autorisation) {
    digitalWrite(POMPE, autorisation);
    _etat_arrosage = autorisation;
}


/**
 * @brief donner l'autorisation d'éclairage selon la programmation horaire. 
 * @param debut  heure de début entre 0 et 24h.
 * @param duree duree d'éclairage en heure entre 1 et 23h. 0 ou > 23 éclairage permanent.
 * @return autorisation d'éclairage.
 */
bool SerreAuto::autoriser_eclairage(uint8_t debut, uint8_t duree) {
    
    bool inversion_plage = false;
    bool validation = false;

    if (duree == 0) {
        return true;
    } 
    else if (duree > 23) {
        return true;
    }
    else if ((duree + debut) > 23) {
        inversion_plage = true;
        debut = debut + duree - 24;
        duree = 24 - duree;
    }
    
    DateTime now = rtc.now();

    if (now.hour() < (debut + duree) ) {
        if(now.hour() >= debut) {
            validation = true;
        }
        else {
            validation = false;
        }
    }

    if (inversion_plage)  return !validation;
    else return validation;
}


/**
 * @brief donner l'autorisation d'arrosage selon la programmation horaire. L'attribut _jour_arrosage_valide est initialisé lors de la mise sous-tension ou d'un reboot au numéro du jour de la semaine courante. Exemple si reboot mardi _jour_arrosage_valide = 2
 * @param debut heure de début de 0 à 23h
 * @param duree duree d'arrosage en seconde approximativement 13s pour 10cl
 * @param periodicite {1, 2, 3} tous les jours, tous les deux jours ...
 * @return autorisation d'arrosage
 * 
 */
bool SerreAuto::autoriser_arrosage(uint8_t debut, uint16_t duree, uint8_t periodicite) {
    uint8_t jour_valide = _jour_arrosage_valide;
    uint8_t prochain_jour_valide;
    static bool depart_timer;
    static long start_time;
    if(debut > 23)  debut = 6;
    if(duree > 60*10) duree = 600;
        else if (duree == 0)  return false;
    if(duree < 5)  duree = 5;
    if(periodicite > 3) periodicite = 3;
        else if(periodicite == 0) return false;
    
    DateTime now = rtc.now();
    DateTime prochain (now + TimeSpan(periodicite,0,0,0));
    
    if(now.dayOfTheWeek() == jour_valide) {
        if (now.hour() == debut) {
            prochain_jour_valide = jour_valide + periodicite;
            if (prochain_jour_valide > 6)  prochain_jour_valide = prochain_jour_valide - 7;
            _jour_arrosage_valide = prochain_jour_valide;
            depart_timer = true;
            start_time = now.unixtime();
        }
    }
    if(depart_timer) {
        if((now.unixtime() - start_time) < duree) {
            return true;
        }
        else {
            //fin d'arrosage
            depart_timer = false;
            return false;
        }
    }
    else  return false;
}


/**
 * @brief Setter pour l'attribut _jour_arrosage_valide
 * @param jour_valide un jour de la semaine compris entre 0 et 6. Dimanche est le jour 0
 * 
 */
void SerreAuto::jour_arrosage_valide(uint8_t jour_valide) {
    _jour_arrosage_valide = jour_valide;
}


/**
 * @brief Getter pour l'attribut _jour_arrosage_valide
 * @return attribut correspondant au jour d'arrosage valide dans la semaine. Compris entre 0 et 6. Dimanche est le jour 0 
 */
uint8_t SerreAuto::jour_arrosage_valide() const {

    return _jour_arrosage_valide;
}


/**
 * @brief Getter de la commande MLI du plateau chauffant
 * 
 */
uint8_t SerreAuto::mli_plateau_chauffant() const {
    return _commande_mli;
}


/**
 * @brief Afficher un message sur le lcd en cas d'arrêt d'urgence
 * 
 */
void SerreAuto::afficher_arret_urgence() const {
    lcd.setCursor(0, 0);
    lcd.print("Arret d'urgence ");
    lcd.setCursor(0, 1);
    lcd.print("Need to fix asap");
}


/**
 * @brief mémorise les paramètres d'arrosage dans la mémoire de l'horloge
 * 
 */
void SerreAuto::enregistrer_parametres_arrosage() {
    uint8_t duree_H = _duree_arrosage >> 8;
    uint8_t duree_L = _duree_arrosage & 0x00FF;
    rtc.writenvram(0, _heure_debut_arrosage);
    rtc.writenvram(1, duree_H);
    rtc.writenvram(2, duree_L);
    rtc.writenvram(3, _periodicite_arrosage);

}


/**
 * @brief restaure les paramètres d'arrosage depuis la mémoire de l'horloge
 * 
 */
void SerreAuto::restaurer_parametres_arrosage() {
    _heure_debut_arrosage = rtc.readnvram(0);
    uint8_t duree_H = rtc.readnvram(1);
    uint8_t duree_L = rtc.readnvram(2);
    _periodicite_arrosage = rtc.readnvram(3);
    _duree_arrosage = (duree_H << 8) + duree_L;
}


SerreAuto::~SerreAuto() {
	// TODO Auto-generated destructor stub
}
