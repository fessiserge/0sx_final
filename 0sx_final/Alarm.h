#pragma once


enum AlarmState {

  OFF,

  WATCHING,

  ON,

  TESTING

};


class Alarm {

public:

  // Constructeur

  // rPin, gPin, bPin : broches pour la DEL RGB

  // buzzerPin : broche du buzzer

  // distancePtr : pointeur vers la variable de distance partagée

  Alarm(int rPin, int gPin, int bPin, int buzzerPin, float* distancePtr);


  // Méthode à appeler continuellement dans loop()

  void update();


  // Régle les deux couleurs du gyrophare

  void setColourA(int r, int g, int b);

  void setColourB(int r, int g, int b);


  // Régle la fréquence de variation du gyrophare (en ms)

  void setVariationTiming(unsigned long ms);


  // Régle la distance de déclenchement (en cm)

  void setDistance(float d);


  // Régle le délai avant l'extinction après éloignement (en ms)

  void setTimeout(unsigned long ms);


  // Éteint/Allume l'alarme manuellement

  void turnOff();

  void turnOn();


  // Déclenche un test de 3 secondes

  void test();


  // Retourne l'état courant de l'alarme

  AlarmState getState() const;


private:

  // --- Broches matérielles ---

  int _rPin, _gPin, _bPin, _buzzerPin;


  unsigned long _currentTime = 0;


  // --- Couleurs du gyrophare ---

  int _colA[3];  // Couleur A

  int _colB[3];  // Couleur B

  bool _currentColor = false;  // false = couleur A, true = couleur B


  // --- Détection ---

  float* _distance;  // Pointeur vers la distance externe

  float _distanceTrigger = 10.0;  // Distance minimale pour déclenchement


  // --- Temporisation ---

  unsigned long _variationRate = 500;  // Intervalle entre deux couleurs

  unsigned long _timeoutDelay = 3000;  // Délai avant arrêt

  unsigned long _lastUpdate = 0;  // Pour le gyrophare

  unsigned long _lastDetectedTime = 0;  // Dernière fois qu'on a vu l'objet

  unsigned long _testStartTime = 0;  // Pour TESTING


  // --- État interne ---

  AlarmState _state = OFF;


  // --- Indicateurs de transition ---

  bool _turnOnFlag = false;  // Pour transition vers ON

  bool _turnOffFlag = false;  // Pour transition vers OFF


  // --- Méthodes internes ---

  void _setRGB(int r, int g, int b);

  void _turnOff();  // Éteint DEL et buzzer


  // Gestion des états

  void _offState();

  void _watchState();

  void _onState();

  void _testingState();
};