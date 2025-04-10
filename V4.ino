#include <Wire.h>
#include <TimerOne.h>
#include <QTRSensors.h>

/////////////////////////////////////////////////////////////////////
//    Variables globales et fonctions pour le suivi de ligne       //
/////////////////////////////////////////////////////////////////////

// Définition des broches pour les moteurs
#define borneENA        10      // Broche ENA du L298N connectée à la broche D10 de l'Arduino (PWM possible)
#define borneIN1        9       // Broche IN1 du L298N connectée à la broche D9 de l'Arduino
#define borneIN2        8       // Broche IN2 du L298N connectée à la broche D8 de l'Arduino
#define borneIN3        7       // Broche IN3 du L298N connectée à la broche D7 de l'Arduino
#define borneIN4        6       // Broche IN4 du L298N connectée à la broche D6 de l'Arduino
#define borneENB        5       // Broche ENB du L298N connectée à la broche D5 de l'Arduino (PWM possible)

// Définition des constantes du programme
#define delaiChangementVitesse     00      // Délai en millisecondes avant de changer la vitesse
#define vitesseMinimale            120     // Rapport cyclique PWM minimal pour le moteur
#define vitesseMaximale            255     // Rapport cyclique PWM maximal pour le moteur
#define vitesseRotation            255     // Rapport cyclique PWM maximal pour la rotation

// Définition des broches pour les capteurs
const int capteurG1 = 24; // Capteur gauche 1
const int capteurG2 = 25; // Capteur gauche 2
const int capteurG3 = 22; // Capteur gauche 3
const int capteurG4 = 23; // Capteur gauche 4
const int capteurD1 = 53; // Capteur droit 1
const int capteurD2 = 52; // Capteur droit 2
const int capteurD3 = 50; // Capteur droit 3
const int capteurD4 = 51; // Capteur droit 4

// Définition des broches pour les moteurs (GA et GB pour le moteur Gauche, DA et DB pour le moteur de droite)
const int moteurGA = 12;
const int moteurGB = 3;
const int moteurDA = 13;
const int moteurDB = 11;

// Définition des capteurs de suivit de ligne en suivant le code de la documentation
const uint8_t SensorCount = 4;

QTRSensors qtrd;
uint16_t sensorValuesD[SensorCount];

QTRSensors qtrg;
uint16_t sensorValuesG[SensorCount];

// Définition des commandes de mouvement
const char MARCHE_AVANT   = 'A';
const char MARCHE_ARRIERE = 'R';
const char STOP           = 'S'; // Ajout d'une commande STOP

int vitesseActuelleGauche = 0; // Utiliser des noms de variables plus descriptifs
int vitesseActuelleDroite = 0;

// Déclaration des fonctions
void configurerSensDeRotationPontA(char sensDeRotation);
void changeVitesseMoteurPontA(int vitesse);
void configurerSensDeRotationPontB(char sensDeRotation);
void changeVitesseMoteurPontB(int vitesse);
void avancer(float facteur);
void reculer(float facteur);
void tournerGauche(float facteur);
void tournerDroite(float facteur);
void tournerGaucheExtremite(float facteur);
void tournerDroiteExtremite(float facteur);
void arreter(); // Ajout d'une fonction arreter

/////////////////////////////////////////////////////////////////////
//   Variables globales et fonctions pour le capteur de couleur    //
/////////////////////////////////////////////////////////////////////

// affichage debug 
bool debug = false;

// Calibration du blanc 
int Ri = 0;
int Bi = 0;
int Vi = 0;
float limBlanc = 0.8;
float limNoir = 0.3;

// Paramètres attachInterrupt
int temps = 1200;
int periode = 1000000;

// Capteur gauche
const int S0g = 29;
const int S1g = 28;
const int S2g = 27;
const int S3g = 26;
const int OUTg = 2;

int Rg = 0;
int Vg = 0;
int Bg = 0;
int g_countg = 0;
int g_arrayg[3];
int g_flagg = 0;
bool isBlueG = false;
bool isRedG = false;
bool isGreenG = false;
bool isWhiteG = false;
bool isBlackG = false;

int Rg0 = 0;
int Vg0 = 0;
int Bg0 = 0;

// Capteur droit
const int S0d = 49;
const int S1d = 48;
const int S2d = 47;
const int S3d = 46;
const int OUTd = 3;
int Rd = 0;
int Vd = 0;
int Bd = 0;
int g_countd = 0;
int g_arrayd[3];
int g_flagd = 0;
bool isBlueD = false;
bool isRedD = false;
bool isGreenD = false;
bool isWhiteD = false;
bool isBlackD = false;

int Rd0 = 0;
int Vd0 = 0;
int Bd0 = 0;

// Déclaration des fonctions
void calibration();
bool isRed(int R,int V,int B);
bool isGreen(int R,int V,int B);
bool isBlue(int R,int V,int B);
bool isWhite(int R,int V,int B);
bool isBlack(int R, int V, int B);
void TSC_Initd();
void TSC_FilterColord(int Level01, int Level02);
void TSC_Countd();
void TSC_WBd(int Level0, int Level1);
void TSC_Callbackd();
void captLumD();
void setupd();
void TSC_Initg();
void TSC_FilterColorg(int Level01, int Level02);
void TSC_Countg();
void TSC_WBg(int Level0, int Level1);
void TSC_Callbackg();
void captLumG();
void setupg();
void mesureg(); // Fonction à appeler pour mesurer la couleur à gauche
void mesured(); // Fonction à appeler pour mesurer la couleur à droite

/////////////////////////////////////////////////////////////////////
//                      Programme principal                        //
/////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);

  //Configuration des capteurs

  qtrd.setTypeRC();
  qtrd.setSensorPins((const uint8_t[]){capteurD1, capteurD2, capteurD3, capteurD4}, SensorCount);
  qtrd.setEmitterPin(31);
  
  qtrg.setTypeRC();
  qtrg.setSensorPins((const uint8_t[]){capteurG1, capteurG2, capteurG3, capteurG4}, SensorCount);
  qtrg.setEmitterPin(31);


  // Configuration des broches des capteurs en entrée
  pinMode(capteurG1, INPUT);
  pinMode(capteurG2, INPUT);
  pinMode(capteurG3, INPUT);
  pinMode(capteurG4, INPUT);
  pinMode(capteurD1, INPUT);
  pinMode(capteurD2, INPUT);
  pinMode(capteurD3, INPUT);
  pinMode(capteurD4, INPUT);

  // Configuration des broches des moteurs en sortie
  pinMode(borneENA, OUTPUT);
  pinMode(borneIN1, OUTPUT);
  pinMode(borneIN2, OUTPUT);
  pinMode(borneIN3, OUTPUT);
  pinMode(borneIN4, OUTPUT);
  pinMode(borneENB, OUTPUT);

  // Configuration des broches des capteurs de couleur
  TSC_Initg();
  TSC_Initd();
  calibration();
}

void loop() {
  // Lecture de l'état des capteurs
  int G4 = digitalRead(capteurG1);
  int G3 = digitalRead(capteurG2);
  int G2 = digitalRead(capteurG3);
  int G1 = digitalRead(capteurG4);
  int D4 = digitalRead(capteurD1);
  int D3 = digitalRead(capteurD2);
  int D2 = digitalRead(capteurD3);
  int D1 = digitalRead(capteurD4);

  // Affichage des valeurs des capteurs pour le débogage
  Serial.print("Capteurs: G4="); Serial.print(G4);
  Serial.print(" G3="); Serial.print(G3);
  Serial.print(" G2="); Serial.print(G2);
  Serial.print(" G1="); Serial.print(G1);
  Serial.print(" D4="); Serial.print(D4);
  Serial.print(" D3="); Serial.print(D3);
  Serial.print(" D2="); Serial.print(D2);
  Serial.print(" D1="); Serial.print(D1);
  Serial.println();

  // Logique de mouvement basée sur les capteurs
  if (D1 && D2 && D3 && D4 && G1 && G2 && G3 && G4) {
    Serial.println("STOP");
    arreter();
    mesured();
    mesureg();
    mesured();
    mesureg();
    if(isGreenD && isWhiteG){
      
    }
    else if(isGreenG && isWhiteD){
      
    }
    else if(isGreenD && isGreenG){
      
    }
    else if(isWhiteD && isWhiteG){
      
    }
  } else if (D4) {
     Serial.println("Tourner à droite 100%");
     //while (digitalRead(capteurG2) == 0 && digitalRead(capteurG1) == 0 && digitalRead(capteurG3) == 0 && digitalRead(capteurG4) == 0) {
        tournerdroite(100);
      //}
   } else if (G4) {
     Serial.println("Tourner à gauche 100%");
     //while (digitalRead(capteurD2) == 0 && digitalRead(capteurD1) == 0 && digitalRead(capteurD3) == 0 && digitalRead(capteurD4) == 0) {
        tournergauche(100);
      //}
   } else if (G3) {
     Serial.println("Tourner à gauche 90%");
     tournergauche(95);
   } else if (D3) {
     Serial.println("Tourner à droite 90%");
     tournerdroite(95);
   } else if (G2) {
     Serial.println("Tourner à gauche 80%");
     tournergauche(92);
   } else if (D2) {
     Serial.println("Tourner à droite 80%");
     tournerdroite(92);
   } else if (G1) {
     Serial.println("Tourner à gauche 70%");
     tournergauche(92);
   } else if (D1) {
     Serial.println("Tourner à droite 70%");
     tournerdroite(92);
   } else if (G1 && D1) {
     Serial.println("Avancer");
     avancer(87);
   } else {
     Serial.println("Avancer");
     avancer(87);
   }
}

/////////////////////////////////////////////////////////////////////
//                   Fonctions pour les moteurs                    //
/////////////////////////////////////////////////////////////////////

// Fonction pour arrêter les moteurs
void arreter() {
  digitalWrite(borneIN1, LOW);
  digitalWrite(borneIN2, LOW);
  digitalWrite(borneIN3, LOW);
  digitalWrite(borneIN4, LOW);
  analogWrite(borneENA, 0);
  analogWrite(borneENB, 0);
}

void arriere(float facteur) {
  facteur = facteur / 100;
  configurerSensDeRotationPontA(MARCHE_AVANT);
  changeVitesseMoteurPontA(vitesseMaximale*facteur);
  configurerSensDeRotationPontB(MARCHE_AVANT);
  changeVitesseMoteurPontB(vitesseMaximale*facteur);
}

void avancer(float facteur) {
  facteur = facteur / 100;
  configurerSensDeRotationPontA(MARCHE_ARRIERE);
  changeVitesseMoteurPontA(vitesseMaximale*facteur);
  configurerSensDeRotationPontB(MARCHE_ARRIERE);
  changeVitesseMoteurPontB(vitesseMaximale*facteur);
}

void tournergauche(float facteur) {  
  facteur = facteur / 100;
  configurerSensDeRotationPontA(MARCHE_ARRIERE);
  changeVitesseMoteurPontA(vitesseRotation*facteur);
  configurerSensDeRotationPontB(MARCHE_AVANT);
  changeVitesseMoteurPontB(vitesseRotation*facteur/2);
  
}
void tournergaucheextremite(float facteur) {  
  facteur = facteur / 100;
  configurerSensDeRotationPontA(MARCHE_ARRIERE);
  changeVitesseMoteurPontA(vitesseRotation * facteur);
  configurerSensDeRotationPontB(MARCHE_AVANT);
  changeVitesseMoteurPontB(vitesseRotation * facteur * 0.7);
}

void tournerdroite(float facteur) {  
  facteur = facteur / 100;
  configurerSensDeRotationPontA(MARCHE_AVANT);
  changeVitesseMoteurPontA(vitesseRotation*facteur/2);
  configurerSensDeRotationPontB(MARCHE_ARRIERE);
  changeVitesseMoteurPontB(vitesseRotation*facteur);
  
}

void tournerdroiteextremite(float facteur) {  
  facteur = facteur / 100;

  configurerSensDeRotationPontA(MARCHE_AVANT);
  changeVitesseMoteurPontA(vitesseRotation * facteur * 0.7);
  configurerSensDeRotationPontB(MARCHE_ARRIERE);
  changeVitesseMoteurPontB(vitesseRotation * facteur);
}

void configurerSensDeRotationPontA(char sensDeRotation) {
  if (sensDeRotation == MARCHE_AVANT) {
    digitalWrite(borneIN1, HIGH);
    digitalWrite(borneIN2, LOW);
  } else {
    digitalWrite(borneIN1, LOW);
    digitalWrite(borneIN2, HIGH);
  }
}

void changeVitesseMoteurPontA(int vitesse) {
  analogWrite(borneENA, vitesse);
}

void configurerSensDeRotationPontB(char sensDeRotation) {
  if (sensDeRotation == MARCHE_AVANT) {
    digitalWrite(borneIN3, HIGH);
    digitalWrite(borneIN4, LOW);
  } else {
    digitalWrite(borneIN3, LOW);
    digitalWrite(borneIN4, HIGH);
  }
}

void changeVitesseMoteurPontB(int vitesse) {
  analogWrite(borneENB, vitesse);
}

/////////////////////////////////////////////////////////////////////
//              Fonctions pour le capteur de couleur               //
/////////////////////////////////////////////////////////////////////

void calibration() {
  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callbackd);
  attachInterrupt(digitalPinToInterrupt(OUTd), TSC_Countd, RISING);
  delay(2 * temps);
  Rd0 = g_arrayd[0]; // valeur R
  Vd0 = g_arrayd[1] ; // valeur V
  Bd0 = g_arrayd[2] ; // valeur B
  Timer1.detachInterrupt();
  detachInterrupt(OUTd);
  Serial.println("Calibration droite");
  Serial.println(Rd0);
  Serial.println(Vd0);
  Serial.println(Bd0);

  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callbackg);
  attachInterrupt(digitalPinToInterrupt(OUTg), TSC_Countg, RISING);
  delay(2 * temps);
  Timer1.detachInterrupt();
  detachInterrupt(OUTg);
  Serial.println("Calibration gauche");
  Rg0 = g_arrayg[0]; // valeur R
  Vg0 = g_arrayg[1] ; // valeur V
  Bg0 = g_arrayg[2] ; // valeur B
  Serial.println(Rg0);
  Serial.println(Vg0);
  Serial.println(Bg0);
  Ri = min(Rg0, Rd0);
  Vi = min(Vg0, Vd0);
  Bi = min(Bg0, Bd0);
  Serial.println("Calibration blanc :");
  Serial.println(Ri);
  Serial.println(Vi);
  Serial.println(Bi);
}

// détection des couleurs
bool isRed(int R,int V,int B) {
  float rp = float(R)/float(Ri);
  float vp = float(V)/float(Vi);
  float bp = float(B)/float(Bi);
  bool isb = rp < limNoir && vp < limNoir && bp < limNoir;
  bool isw = rp > limBlanc && vp > limBlanc && bp > limBlanc;
  return (rp > vp && rp > bp && !isb && !isw);
}

bool isGreen(int R,int V,int B) {
  float rp = float(R)/float(Ri);
  float vp = float(V)/float(Vi);
  float bp = float(B)/float(Bi);
  bool isb = rp < limNoir && vp < limNoir && bp < limNoir;
  bool isw = rp > limBlanc && vp > limBlanc && bp > limBlanc;
  return (vp > rp && vp > bp && !isb && !isw);
}

bool isBlue(int R,int V,int B) {
  float rp = float(R)/float(Ri);
  float vp = float(V)/float(Vi);
  float bp = float(B)/float(Bi);
  bool isb = rp < limNoir && vp < limNoir && bp < limNoir;
  bool isw = rp > limBlanc && vp > limBlanc && bp > limBlanc;
  return (bp > rp && bp > vp && !isb && !isw);
}

bool isWhite(int R,int V,int B) {
  float rp = float(R)/float(Ri);
  float vp = float(V)/float(Vi);
  float bp = float(B)/float(Bi);
  bool isb = rp < limNoir && vp < limNoir && bp < limNoir;
  bool isw = rp > limBlanc && vp > limBlanc && bp > limBlanc;
  return (isw);
}

bool isBlack(int R, int V, int B){
  float rp = float(R)/float(Ri);
  float vp = float(V)/float(Vi);
  float bp = float(B)/float(Bi);
  bool isb = rp < limNoir && vp < limNoir && bp < limNoir;
  bool isw = rp > limBlanc && vp > limBlanc && bp > limBlanc;
  return (isb);
}

// fonctions pour capteur droit
void TSC_Initd()
{
  pinMode(S0d, OUTPUT);
  pinMode(S1d, OUTPUT);
  pinMode(S2d, OUTPUT);
  pinMode(S3d, OUTPUT);
  pinMode(OUTd, INPUT);
  digitalWrite(S0d, LOW);
  digitalWrite(S1d, HIGH);
}

void TSC_FilterColord(int Level01, int Level02)
{
  if (Level01 != 0)
    Level01 = HIGH;
  if (Level02 != 0)
    Level02 = HIGH;
  digitalWrite(S2d, Level01);
  digitalWrite(S3d, Level02);
}

void TSC_Countd()
{
  g_countd ++;
}

void TSC_WBd(int Level0, int Level1) // Balance des blancs
{
  g_countd = 0;
  g_flagd ++;
  TSC_FilterColord(Level0, Level1);
  Timer1.setPeriod(periode);
}

void TSC_Callbackd()
{
  switch (g_flagd)
  {
    case 0:
      if (debug) {
        Serial.println("->WB Start droite");
      }
      TSC_WBd(LOW, LOW); // Filtre sans rouge
      break;
    case 1:
      if (debug) {
        Serial.print("->Frequency R=");
        Serial.println(g_countd);
      }
      g_arrayd[0] = g_countd;
      TSC_WBd(HIGH, HIGH); // Filtre sans vert
      break;
    case 2:
      if (debug) {
        Serial.print("->Frequency V=");
        Serial.println(g_countd);
      }
      g_arrayd[1] = g_countd;
      TSC_WBd(LOW, HIGH); // Filtre sans bleu
      break;
    case 3:
      if (debug) {
        Serial.print("->Frequency B=");
        Serial.println(g_countd);
        Serial.println("->WB End");
      }
      g_arrayd[2] = g_countd;
      TSC_WBd(HIGH, LOW); // Pas de filtre
      break;
    default:
      g_countd = 0;
      break;
  }
}

void captLumD() {
  g_flagd = 0;
  Rd = g_arrayd[0]; 
  Vd = g_arrayd[1]; 
  Bd = g_arrayd[2];

  Serial.print("Couleur détectée à droite : ");
  if (isRed(Rd,Vd,Bd)) {
    Serial.println("Rouge");
    isRedD = true;
    isGreenD = false;
    isBlueD = false;
    isWhiteD = false;
    isBlackD = false;
  }
  else if (isGreen(Rd,Vd,Bd)) {
    Serial.println("Vert");
    isRedD = false;
    isGreenD = true;
    isBlueD = false;
    isWhiteD = false;
    isBlackD = false;
  }
  else if (isBlue(Rd,Vd,Bd)) {
    Serial.println("Bleu");
    isRedD = false;
    isGreenD = false;
    isBlueD = true;
    isWhiteD = false;
    isBlackD = false;
  }
  else if (isWhite(Rd,Vd,Bd)) {
    Serial.println("Blanc");
    isRedD = false;
    isGreenD = false;
    isBlueD = false;
    isWhiteD = true;
    isBlackD = false;
  }
  else if(isBlack(Rd,Vd,Bd)){
    Serial.println("Noir");
    isRedD = false;
    isGreenD = false;
    isBlueD = false;
    isWhiteD = false;
    isBlackD = true;
  }
  else {
    Serial.println("Non fiable");
  }
}

void setupd() {
  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callbackd);
  attachInterrupt(digitalPinToInterrupt(OUTd), TSC_Countd, RISING);
  delay(temps);
  if (debug) {
    for (int i = 0; i < 3; i++)
      Serial.println(g_arrayd[i]);
  }
}

// fonctions pour capteur gauche
void TSC_Initg()
{
  pinMode(S0g, OUTPUT);
  pinMode(S1g, OUTPUT);
  pinMode(S2g, OUTPUT);
  pinMode(S3g, OUTPUT);
  pinMode(OUTg, INPUT);
  digitalWrite(S0g, LOW);
  digitalWrite(S1g, HIGH);
}

void TSC_FilterColorg(int Level01, int Level02)
{
  if (Level01 != 0)
    Level01 = HIGH;
  if (Level02 != 0)
    Level02 = HIGH;
  digitalWrite(S2g, Level01);
  digitalWrite(S3g, Level02);
}

void TSC_Countg()
{
  g_countg ++;
}

void TSC_WBg(int Level0, int Level1) // Balance des blancs
{
  g_countg = 0;
  g_flagg ++;
  TSC_FilterColorg(Level0, Level1);
  Timer1.setPeriod(periode);
}

void TSC_Callbackg()
{
  switch (g_flagg)
  {
    case 0:
      if (debug) {
        Serial.println("->WB Start gauche");
      }
      TSC_WBg(LOW, LOW); // Filtre sans rouge
      break;
    case 1:
      if (debug) {
        Serial.print("->Frequency R=");
        Serial.println(g_countg);
      }
      g_arrayg[0] = g_countg;
      TSC_WBg(HIGH, HIGH); // Filtre sans vert
      break;
    case 2:
      if (debug) {
        Serial.print("->Frequency V=");
        Serial.println(g_countg);
      }
      g_arrayg[1] = g_countg;
      TSC_WBg(LOW, HIGH); // Filtre sans bleu
      break;
    case 3:
      if (debug) {
        Serial.print("->Frequency B=");
        Serial.println(g_countg);
        Serial.println("->WB End");
      }
      g_arrayg[2] = g_countg;
      TSC_WBg(HIGH, LOW); // Pas de filtre
      break;
    default:
      g_countg = 0;
      break;
  }
}

void captLumG() {
  g_flagg = 0;
  Rg = g_arrayg[0];
  Vg = g_arrayg[1];
  Bg = g_arrayg[2];

  Serial.print("Couleur détectée à gauche : ");
  if (isRed(Rg,Vg,Bg)) {
    Serial.println("Rouge");
    isRedG = true;
    isGreenG = false;
    isBlueG = false;
    isWhiteG = false;
    isBlackG = false;
  }
  else if (isGreen(Rg,Vg,Bg)) {
    Serial.println("Vert");
    isRedG = false;
    isGreenG = true;
    isBlueG = false;
    isWhiteG = false;
    isBlackG = false;
  }
  else if (isBlue(Rg,Vg,Bg)) {
    Serial.println("Bleu");
    isRedG = false;
    isGreenG = false;
    isBlueG = true;
    isWhiteG = false;
    isBlackG = false;
  }

  else if (isWhite(Rg,Vg,Bg)) {
    Serial.println("Blanc");
    isRedG = false;
    isGreenG = false;
    isBlueG = false;
    isWhiteG = true;
    isBlackG = false;
  }
  else if (isBlack(Rg,Vg,Bg)) {
    Serial.println("Blanc");
    isRedG = false;
    isGreenG = false;
    isBlueG = false;
    isWhiteG = false;
    isBlackG = true;
  }
  else {
    Serial.println("Non fiable");
  }
}

void setupg() {
  Timer1.initialize();
  Timer1.attachInterrupt(TSC_Callbackg);
  attachInterrupt(digitalPinToInterrupt(OUTg), TSC_Countg, RISING);
  delay(temps);
  if (debug) {
    for (int i = 0; i < 3; i++)
      Serial.println(g_arrayg[i]);
  }
}

// Fonction à appeler dans le main pour mesurer la couleur à droite
void mesured() {
  setupd();
  captLumD();
  delay(temps);
}
// Fonction à appeler dans le main pour mesurer la couleur à gauche
void mesureg() {
  setupg();
  captLumG();
  delay(temps);
}
