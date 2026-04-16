#include <Wire.h>
#include <TimerOne.h>
#include <QTRSensors.h>

/////////////////////////////////////////////////////////////////////
//       Variables globales et fonctions pour les moteurs          //
/////////////////////////////////////////////////////////////////////

// Définition des broches pour les moteurs
#define borneENA 10   // Broche ENA du L298N connectée à la broche D10 de l'Arduino (PWM possible)
#define borneIN1 9    // Broche IN1 du L298N connectée à la broche D9 de l'Arduino
#define borneIN2 8    // Broche IN2 du L298N connectée à la broche D8 de l'Arduino
#define borneIN3 7    // Broche IN3 du L298N connectée à la broche D7 de l'Arduino
#define borneIN4 6    // Broche IN4 du L298N connectée à la broche D6 de l'Arduino
#define borneENB 5    // Broche ENB du L298N connectée à la broche D5 de l'Arduino (PWM possible)
#define tolerance 25  // 0 avant
// Définition des commandes de mouvement
const char MARCHE_AVANT = 'R';
const char MARCHE_ARRIERE = 'A';

// Déclaration des fonctions liées aux moteurs
void configurerSensDeRotationPontA(char sensDeRotation);
void changeVitesseMoteurPontA(int vitesse);
void configurerSensDeRotationPontB(char sensDeRotation);
void changeVitesseMoteurPontB(int vitesse);
void arreter();

/////////////////////////////////////////////////////////////////////
//    Variables globales et fonctions pour le suivi de ligne       //
/////////////////////////////////////////////////////////////////////

// Définition des broches pour les capteurs n&b
const int capteur1 = 34;  // Capteur gauche 1
const int capteur2 = 35;  // Capteur gauche 2
const int capteur3 = 36;  // Capteur gauche 3
const int capteur4 = 37;  // Capteur gauche 4
const int capteur5 = 38;  // Capteur droit 1
const int capteur6 = 39;  // Capteur droit 2

// Définition des variables du capteur de suivit de ligne

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
int lastError = 0;

// Variables liées à l'asservissement
const int MAX_SPEED = 190;  //220 ou 200 avant
const float Kp = 3;         //2 avant
const float Ki = 0;
const float Kd = 3;

/////////////////////////////////////////////////////////////////////
//       Variables globales et fonctions pour les couleurs         //
/////////////////////////////////////////////////////////////////////

// Définition des broches pour les capteurs de couleurs
#define S0 53   // sur borne digitale
#define S1 52   // sur borne digitale
#define S2 51   // sur borne digitale
#define S3 50   // sur borne digitale
#define OUTd 3  // sur borne attachInterrupt(2 ou 3)
#define OUTg 2  // sur borne attachInterrupt(2 ou 3)
bool debug = false;
bool isBlue, isRed, isGreenD, isGreenG;

const int frequence = 2;  // choisir 2%, 20% ou 100%
float periode = 0;        // en microsecondes
float attente = 0;        // en millisecondes
int g_flag = 0;

int g_countg = 0;
int g_arrayg[3];
float g_SFg[3];
int Rg, Gg, Bg, R0g, G0g, B0g;

int g_countd = 0;
int g_arrayd[3];
float g_SFd[3];
int Rd, Gd, Bd, R0d, G0d, B0d;



/////////////////////////////////////////////////////////////////////
//                      Programme principal                        //
/////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);

  // Configuration des capteurs de suivi de ligne
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      capteur1, capteur2, capteur3, capteur4, capteur5, capteur6 },
                    SensorCount);
  qtr.setEmitterPin(4);

  pinMode(LED_BUILTIN, OUTPUT);


  // Calibration des capteurs

  TSC_Init();
  Timer1.initialize(periode);  // periode en microsecondes
  Timer1.attachInterrupt(TSC_Callback);
  attachInterrupt(digitalPinToInterrupt(OUTg), TSC_Countg, RISING);
  attachInterrupt(digitalPinToInterrupt(OUTd), TSC_Countd, RISING);
  delay(attente);

  // Calibration gauche
  R0g = g_arrayg[0];
  G0g = g_arrayg[1];
  B0g = g_arrayg[2];
  g_SFg[0] = 255.0 / g_arrayg[0];  // valeur R
  g_SFg[1] = 255.0 / g_arrayg[1];  // valeur G
  g_SFg[2] = 255.0 / g_arrayg[2];  // valeur B

  // Calibration droite
  R0d = g_arrayd[0];
  G0d = g_arrayd[1];
  B0d = g_arrayd[2];
  g_SFd[0] = 255.0 / g_arrayd[0];  // valeur R
  g_SFd[1] = 255.0 / g_arrayd[1];  // valeur G
  g_SFd[2] = 255.0 / g_arrayd[2];  // valeur B

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Début de la calibration des capteurs de suivit de ligne");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

  // Configuration des broches des moteurs en sortie
  pinMode(borneENA, OUTPUT);
  pinMode(borneIN1, OUTPUT);
  pinMode(borneIN2, OUTPUT);
  pinMode(borneIN3, OUTPUT);
  pinMode(borneIN4, OUTPUT);
  pinMode(borneENB, OUTPUT);
}

void loop() {


  //Lecture et affichage des valeurs
  qtr.read(sensorValues);
  int position = qtr.readLineBlack(sensorValues);

  if (debug) {
    // print the sensor values as numbers from 0 to 2500, where 0 means maximum
    // reflectance and 2500 means minimum reflectance
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print('\t');

    Serial.print('\t');
    Serial.print('\t');
    Serial.print('\t');


    Serial.print(position);

    Serial.println();
  }

  unsigned int sensors[6];

  if (sensorValues[0] >= 150 && sensorValues[5] >= 150) {
    Serial.print("STOP");
    arreter();
    delay(1000);

    capterCouleur();

    capterCouleur();
    if (isGreenD && !isGreenG) {
      Serial.print("VERT PARTOUT");
    } else if (isGreenG && !isGreenD) {
      Serial.print("VERT GAUCHE");
    } else if (isGreenD && isGreenG) {
      Serial.print("VERT DROITE");
    } else if (!isGreenD && !isGreenG) {
      Serial.print("VERT NULPART");
    }

  } else if (sensorValues[0] <= 50 && sensorValues[1] <= 50 && sensorValues[2] <= 50 && sensorValues[3] <= 50 && sensorValues[4] <= 50 && sensorValues[5] <= 50) {
    configurerSensDeRotationPontA(MARCHE_AVANT);
    configurerSensDeRotationPontB(MARCHE_AVANT);
    changeVitesseMoteurPontA(MAX_SPEED);
    changeVitesseMoteurPontB(MAX_SPEED);
  } else {

    // Get the position of the line.  Note that we *must* provide the "sensors"
    // argument to readLine() here, even though we are not interested in the
    // individual sensor readings          POSITION DEJA DEFINIE PLUS HAUT...

    int m1Speed = 0;
    int m2Speed = 0;

    if (position > 2500 - tolerance && position < 2500 + tolerance) {
      m1Speed = MAX_SPEED;
      m2Speed = MAX_SPEED;
    } else {

      // Our "error" is how far we are away from the center of the line, which
      // corresponds to position 2500.
      int error = position - 2500;

      // Get motor speed difference using proportional and derivative PID terms
      // (the integral term is generally not very useful for line following).
      // Here we are using a proportional constant of 1/4 and a derivative
      // constant of 6, which should work decently for many Zumo motor choices.
      // You probably want to use trial and error to tune these constants for
      // your particular Zumo and line course.
      int speedDifference = Kp * error + Ki * (error + lastError) + Kd * (error - lastError);

      lastError = error;



      // Get individual motor speeds.  The sign of speedDifference
      // determines if the robot turns left or right.


      m1Speed = MAX_SPEED + speedDifference;
      m2Speed = MAX_SPEED - speedDifference;
    }

    configurerSensDeRotationPontA(MARCHE_AVANT);
    configurerSensDeRotationPontB(MARCHE_AVANT);

    if (m1Speed < 0)
      configurerSensDeRotationPontB(MARCHE_ARRIERE);
    m1Speed = abs(m1Speed);
    if (m2Speed < 0)
      configurerSensDeRotationPontA(MARCHE_ARRIERE);
    m2Speed = abs(m2Speed);
    if (m1Speed > MAX_SPEED)
      m1Speed = MAX_SPEED;
    if (m2Speed > MAX_SPEED)
      m2Speed = MAX_SPEED;

    changeVitesseMoteurPontA(m2Speed);
    changeVitesseMoteurPontB(m1Speed);
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


void configurerSensDeRotationPontA(char sensDeRotation) {
  if (sensDeRotation == MARCHE_AVANT) {
    digitalWrite(borneIN1, LOW);
    digitalWrite(borneIN2, HIGH);
  } else {
    digitalWrite(borneIN1, HIGH);
    digitalWrite(borneIN2, LOW);
  }
}

void changeVitesseMoteurPontA(int vitesse) {
  analogWrite(borneENA, vitesse);
}

void configurerSensDeRotationPontB(char sensDeRotation) {
  if (sensDeRotation == MARCHE_AVANT) {
    digitalWrite(borneIN3, LOW);
    digitalWrite(borneIN4, HIGH);
  } else {
    digitalWrite(borneIN3, HIGH);
    digitalWrite(borneIN4, LOW);
  }
}

void changeVitesseMoteurPontB(int vitesse) {
  analogWrite(borneENB, vitesse);
}

/////////////////////////////////////////////////////////////////////
//                   Fonctions pour les couleurs                   //
/////////////////////////////////////////////////////////////////////

void TSC_Init() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUTg, INPUT);
  pinMode(OUTd, INPUT);
  switch (frequence) {
    case 2:
      digitalWrite(S0, LOW);
      digitalWrite(S1, HIGH);
      periode = 1000000;
      break;
    case 20:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
      periode = 100000;
      break;
    case 100:
      digitalWrite(S0, HIGH);
      digitalWrite(S1, HIGH);
      periode = 20000;
      break;
  }
  attente = periode * 4 / 1000;
}
void TSC_FilterColor(int Level01, int Level02) {
  if (Level01 != 0)
    Level01 = HIGH;
  if (Level02 != 0)
    Level02 = HIGH;
  digitalWrite(S2, Level01);
  digitalWrite(S3, Level02);
}
void TSC_Countg() {
  g_countg++;
}
void TSC_Countd() {
  g_countd++;
}
void TSC_Callback() {
  switch (g_flag) {
    case 0:
      if (debug) Serial.println("->WB Start");
      TSC_WB(LOW, LOW);  // Filtre sans rouge
      break;
    case 1:
      if (debug) {
        Serial.print("->Left Frequency R=");
        Serial.print(g_countg);
        Serial.print("    |->Right Frequency R=");
        Serial.println(g_countd);
      }
      g_arrayg[0] = g_countg;
      g_arrayd[0] = g_countd;
      TSC_WB(HIGH, HIGH);  // Filtre sans vert
      break;
    case 2:
      if (debug) {
        Serial.print("->Left Frequency G=");
        Serial.print(g_countg);
        Serial.print("    |->Right Frequency G=");
        Serial.println(g_countd);
      }
      g_arrayg[1] = g_countg;
      g_arrayd[1] = g_countd;
      TSC_WB(LOW, HIGH);  // Filtre sans bleu
      break;
    case 3:
      if (debug) {
        Serial.print("->Left Frequency B=");
        Serial.print(g_countg);
        Serial.print("    |->Right Frequency B=");
        Serial.println(g_countd);
        Serial.println("->WB End");
      }
      g_arrayg[2] = g_countg;
      g_arrayd[2] = g_countd;
      TSC_WB(HIGH, LOW);  // Pas de filtre
      break;
    default:
      g_countg = 0;
      g_countd = 0;
      break;
  }
}
void TSC_WB(int Level0, int Level1)  // Balance des blancs
{
  g_countd = 0;
  g_countg = 0;
  g_flag++;
  TSC_FilterColor(Level0, Level1);
}

void capterCouleur() {
  g_flag = 0;
  Rd = int(g_arrayd[0] * g_SFd[0]);
  Gd = int(g_arrayd[1] * g_SFd[1]);
  Bd = int(g_arrayd[2] * g_SFd[2]);

  Rg = int(g_arrayg[0] * g_SFg[0]);
  Gg = int(g_arrayg[1] * g_SFg[1]);
  Bg = int(g_arrayg[2] * g_SFg[2]);

  Serial.print("Couleur détectée à gauche : ");
  if (Rg > Gg && Rg > Bg) {
    Serial.print("rouge");
    isRed = true;
    isBlue = false;
    isGreenG = false;
  }
  if (Bg > Gg && Bg > Rg) {
    Serial.print("bleu");
    isRed = false;
    isBlue = true;
    isGreenG = false;
  }
  if (Gg > Rg && Gg > Bg) {
    Serial.print("vert");
    isRed = false;
    isBlue = false;
    isGreenG = true;
  }
  Serial.print("    Couleur détectée à droite : ");
  if (Rd > Gd && Rd > Bd) {
    Serial.println("rouge");
    isRed = true;
    isBlue = false;
    isGreenD = false;
  }
  if (Bd > Gd && Bd > Rd) {
    Serial.println("bleu");
    isRed = false;
    isBlue = true;
    isGreenD = false;
  }
  if (Gd > Rd && Gd > Bd) {
    Serial.println("vert");
    isRed = false;
    isBlue = false;
    isGreenD = true;
  }
  if (debug) {
    Serial.print("rouge gauche : ");
    Serial.print(Rg);
    Serial.print("    rouge droite : ");
    Serial.println(Rd);
    Serial.print("vert  gauche : ");
    Serial.print(Gg);
    Serial.print("    vert  droite : ");
    Serial.println(Gd);
    Serial.print("bleu  gauche : ");
    Serial.print(Bg);
    Serial.print("    bleu  droite : ");
    Serial.println(Bd);
    Serial.println("");
  }
  //  delay(attente);
}