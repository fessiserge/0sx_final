
#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>

#include <WiFiEspAT.h>
#include <PubSubClient.h>

#include "Alarm.h"
#include "PorteAutomatique.h"

const int potentiometrePin = A0;
#define HAS_SECRETS 0

#if HAS_SECRETS
#include "arduino_secrets.h"
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
#else
const char ssid[] = "TechniquesInformatique-Etudiant";
const char pass[] = "shawi123";
#endif

WiFiClient espClient;
PubSubClient Client(espClient);
char mqttTopic[] = "etd/12/data";
const int echelle = 100;
const int potentiometreMax = 1023;
int valeurDeDepart = 0;
unsigned long lastMqttSend1 = 0;
unsigned long lastMqttSend2 = 0;
const unsigned long MQTT_SEND_INTERVAL1 = 2500;
const unsigned long MQTT_SEND_INTERVAL2 = 1100;
unsigned long lastMqttAttempt = 0;
bool moteurActif = true;


// === Définition des broches ===
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define BUZZER_PIN 22
#define RED_PIN 10
#define GREEN_PIN 9
#define BLUE_PIN 8
#define IN_1 39
#define IN_2 41
#define IN_3 43
#define IN_4 45
#define CLK_PIN 35
#define DIN_PIN 31
#define CS_PIN 33

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

float distance = 0.0;
unsigned long currentTime = 0;
const char* DA = "2405238";

enum AffichageEtat {
  AUCUN,
  AFFICHAGE_CONFIRMATION,
  AFFICHAGE_ERREUR,
  AFFICHAGE_INCONNU
};
AffichageEtat etatAffichage = AUCUN;

Alarm alarm(RED_PIN, GREEN_PIN, BLUE_PIN, BUZZER_PIN, &distance);
PorteAutomatique porte(IN_1, IN_2, IN_3, IN_4, distance);

unsigned long debutAffichage = 0;
const unsigned long DUREE_AFFICHAGE = 3000;
const int DISPLAY_INTERVAL = 100;
const int DISTANCE_INTERVAL = 50;
const int zero = 0;
int Limite_inferieure = 20;
int Limite_superieure = 30;
bool dirtyFlag1 = true;
bool dirtyFlag2 = true;

float lastDistance = -1;
float lastAngle = -1;
int lastMotor = -1;
int lastPotValue = -1;

char lastLine1[32] = "";
char lastLine2[32] = "";


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.setContrast(5);

  porte.setAngleFerme(10);
  porte.setAngleOuvert(170);

  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  alarm.setDistance(15);
  alarm.setTimeout(2000);
  alarm.setVariationTiming(150);

  Serial3.begin(115200);
  WiFi.init(Serial3);
  WiFi.setPersistent();
  WiFi.endAP();
  WiFi.disconnect();

  Serial.print("Connexion au WiFi ");
  Serial.println(ssid);
  int status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED) {
    Serial.println("WiFi connecté !");
    IPAddress ip = WiFi.localIP();
    Serial.print("Adresse IP : ");
    Serial.println(ip);
  } else {
    Serial.println("Connexion WiFi échouée.");
  }

  Client.setServer("arduino.nb.shawinigan.info", 1883);
  Client.setCallback(mqttEvent);
  Client.subscribe("etd/12/motor", 0);
  Client.subscribe("etd/12/color", 0);

  departureDisplay();
}

void loop() {
  currentTime = millis();
  Client.loop();
  distanceTask();
  if (moteurActif) {
    porte.update();
  }
  alarm.update();
  screenDisplay();
  commandGestion();
  gererAffichageU8g2();

  if (!Client.connected()) reconnectMQTT();


  int motor = (porte.getAngle() > 10) ? 1 : 0;
  int potValue = map(analogRead(potentiometrePin), valeurDeDepart, potentiometreMax, valeurDeDepart, echelle);

  if (distance != lastDistance || porte.getAngle() != lastAngle || motor != lastMotor || potValue != lastPotValue) {
    dirtyFlag1 = true;
    lastDistance = distance;
    lastAngle = porte.getAngle();
    lastMotor = motor;
    lastPotValue = potValue;
  }

  char ligne1[32], ligne2[32];
  char distStr[8];
  dtostrf(distance, 4, 1, distStr);
  snprintf(ligne1, sizeof(ligne1), "Dist: %s cm", distStr);
  snprintf(ligne2, sizeof(ligne2), "Porte: %s", porte.getEtatTexte());

  if (strcmp(ligne1, lastLine1) != 0 || strcmp(ligne2, lastLine2) != 0) {
    dirtyFlag2 = true;
    strncpy(lastLine1, ligne1, sizeof(lastLine1));
    strncpy(lastLine2, ligne2, sizeof(lastLine2));
  }

  if (dirtyFlag1 && millis() - lastMqttSend1 > MQTT_SEND_INTERVAL1) {
    sendMQTTData1();
    lastMqttSend1 = millis();
    dirtyFlag1 = false;
  }

  if (dirtyFlag2 && millis() - lastMqttSend2 > MQTT_SEND_INTERVAL2) {
    sendMQTTData2(ligne1, ligne2);
    lastMqttSend2 = millis();
    dirtyFlag2 = false;
  }
}

void reconnectMQTT() {
  if (millis() - lastMqttAttempt < 5000) return;

  lastMqttAttempt = millis();

  if (!Client.connected()) {
    Serial.print("Connexion MQTT...");
    if (Client.connect("2405238", "etdshawi", "shawi123")) {
      Serial.println("Connecté");

      Client.subscribe("etd/12/motor", 0);
      Client.subscribe("etd/12/color", 0);
      Client.publish("etd/12/motor", "{\"motor\":1}");
      Serial.println("Réabonné aux topics.");
    } else {
      Serial.print("Échec : ");
      Serial.println(Client.state());
    }
  }
}


void sendMQTTData1() {
  if (!Client.connected()) {
    Serial.println("MQTT non connecté, envoi annulé.");
    return;
  }

  static char message[250];
  char distStr[8], angleStr[8];
  int motor = (porte.getAngle() > 10) ? 1 : 0;
  int potValue = map(analogRead(potentiometrePin), valeurDeDepart, potentiometreMax, valeurDeDepart, echelle);  // ← Lire le potentiomètre

  dtostrf(distance, 4, 1, distStr);           // exemple "42.3"
  dtostrf(porte.getAngle(), 4, 1, angleStr);  // exemple "90.0"

  // Ajout de "pot" dans le JSON
  sprintf(message,
          "{\"number\":\"2405238\",\"name\":\"serge landry\", \"uptime\":%lu, \"dist\":%s, \"angle\":%s, \"pot\":%d}",
          millis() / 1000, distStr, angleStr, potValue);

  Serial.print("Message envoyé : ");
  Serial.println(message);

  Client.publish("etd/12/data", message);
}

void sendMQTTData2(const char* ligne1, const char* ligne2) {
  if (!Client.connected()) {
    Serial.println("MQTT non connecté, envoi LCD annulé.");
    return;
  }

  static char message[128];
  snprintf(message, sizeof(message),
           "{\"line1\":\"%s\", \"line2\":\"%s\"}", ligne1, ligne2);

  Serial.print("Message LCD envoyé : ");
  Serial.println(message);

  Client.publish("etd/12/data", message);
}








// Gestion des messages reçues de la part du serveur MQTT
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message recu [");
  Serial.print(topic);
  Serial.print("] ");

  // Convertir le payload en String
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // === GESTION MOTEUR ===
  if (strcmp(topic, "etd/12/motor") == 0) {
    int idx = message.indexOf("\"motor\":");
    if (idx != -1) {
      int val = message.substring(idx + 8).toInt();
      Serial.print("Commande moteur reçue : ");
      Serial.println(val);
      if (val == 1) {
        porte.enableOutput();
        moteurActif = true;  // Active le moteur
        Serial.println("Moteur activé via MQTT");
      } else if (val == 0) {
        moteurActif = false;  // Désactive le moteur
        porte.disableOutput();
        Serial.println("Moteur désactivé via MQTT");
      }
    } else {
      Serial.println("Champ 'motor' non trouvé");
    }
    return;
  }



  // === GESTION COULEUR ===
  if (strcmp(topic, "etd/12/color") == 0) {
    int colorIndex = message.indexOf("#");
    if (colorIndex != -1 && message.length() >= colorIndex + 7) {
      String hexColor = message.substring(colorIndex + 1, colorIndex + 7);  // exemple "FF0000"
      long number = strtol(hexColor.c_str(), NULL, 16);
      int r = (number >> 16) & 0xFF;
      int g = (number >> 8) & 0xFF;
      int b = number & 0xFF;
      alarm.setColourA(r, g, b);
      Serial.print("Nouvelle couleur : ");
      Serial.print(r);
      Serial.print(", ");
      Serial.print(g);
      Serial.print(", ");
      Serial.println(b);

    } else {
      Serial.println("Format de couleur invalide");
    }
    return;
  }

  // Topic non reconnu
  Serial.println(" Topic non reconnu");
}




void gererAffichageU8g2() {
  if (etatAffichage == AUCUN) return;
  if (millis() - debutAffichage > DUREE_AFFICHAGE) {
    u8g2.clearDisplay();
    etatAffichage = AUCUN;
    return;
  }
  u8g2.clearBuffer();
  switch (etatAffichage) {
    case AFFICHAGE_CONFIRMATION:
      u8g2.drawLine(1, 5, 3, 7);
      u8g2.drawLine(3, 7, 7, 1);
      break;
    case AFFICHAGE_ERREUR:
      u8g2.drawCircle(3, 3, 3);
      u8g2.drawLine(0, 0, 7, 7);
      break;
    case AFFICHAGE_INCONNU:
      u8g2.drawLine(7, 7, 0, 0);
      u8g2.drawLine(0, 7, 7, 0);
      break;
    default: break;
  }
  u8g2.sendBuffer();
}

void departureDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DA);
  lcd.setCursor(0, 1);
  lcd.print("Labo 07");
  delay(2000);
  lcd.clear();
}

void distanceTask() {
  static unsigned long last = 0;
  if (millis() - last >= DISTANCE_INTERVAL) {
    float d = hc.dist();
    if (d > 1 && d < 400) {  // HC-SR04 fiable entre ~2 cm et 400 cm
      distance = d;
    }
    last = millis();
  }
}


void screenDisplay() {
  static unsigned long last = 0;
  if (millis() - last >= DISPLAY_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");
    lcd.setCursor(0, 1);
    lcd.print("Porte: ");
    lcd.print(porte.getEtatTexte());
    last = millis();
  }
}

void confirmer() {
  etatAffichage = AFFICHAGE_CONFIRMATION;
  debutAffichage = millis();
}
void erreur() {
  etatAffichage = AFFICHAGE_ERREUR;
  debutAffichage = millis();
}
void inconnu() {
  etatAffichage = AFFICHAGE_INCONNU;
  debutAffichage = millis();
}

void commandGestion() {
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  if (command == "g_dist") {
    Serial.print("Arduino: ");
    Serial.println(distance);
    confirmer();
  } else if (command.startsWith("cfg;alm;")) {
    int val = command.substring(8).toInt();
    if (val > zero) {
      alarm.setDistance(val);
      confirmer();
    } else erreur();
  } else if (command.startsWith("cfg;lim_inf;")) {
    int val = command.substring(12).toInt();
    if (val < Limite_superieure && val > zero) {
      Limite_inferieure = val;
      porte.setDistanceOuverture(val);
      confirmer();
    } else erreur();
  } else if (command.startsWith("cfg;lim_sup;")) {
    int val = command.substring(12).toInt();
    if (val > Limite_inferieure) {
      Limite_superieure = val;
      porte.setDistanceFermeture(val);
      confirmer();
    } else erreur();
  } else inconnu();
}
