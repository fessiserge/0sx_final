#pragma once


#include <AccelStepper.h>


enum EtatPorte {

  FERMEE,

  OUVERTE,

  EN_FERMETURE,

  EN_OUVERTURE

};


class PorteAutomatique {

public:

  // Constructeur

  // p1 à p4 : broches IN1 à IN4 du ULN2003

  // distancePtr : pointeur vers la variable distance partagée

  PorteAutomatique(int p1, int p2, int p3, int p4, float& distancePtr);


  // Doit être appelée continuellement dans loop()

  void update();


  // Régle l'angle maximal d'ouverture de la porte

  void setAngleOuvert(float angle);


  // Régle l'angle de fermeture (habituellement 0°)

  void setAngleFerme(float angle);


  // Définit le nombre de pas par tour du moteur

  void setPasParTour(int steps);


  // Définit la distance de déclenchement pour l'ouverture

  void setDistanceOuverture(float distance);


  // Définit la distance de déclenchement pour la fermeture

  void setDistanceFermeture(float distance);


  // Retourne l'état actuel de la porte sous forme de texte

  const char* getEtatTexte() const;


  // Retourne l'angle actuel de la porte

  float getAngle() const;
  void enableOutput();
  void disableOutput();



private:

  // Moteur AccelStepper

  AccelStepper _stepper;

  unsigned long _currentTime = 0;


  // Angle de position

  float _angleOuvert = 90;

  float _angleFerme = 0;


  // Conversion : pas par tour

  int _stepsPerRev = 2048;  // par défaut pour 28BYJ-48


  // Distance mesurée

  float& _distance;


  // Distance de seuils

  float _distanceOuverture = 20;

  float _distanceFermeture = 30;


  // État actuel

  EtatPorte _etat = FERMEE;


  // Méthodes d'état

  void _ouvertState();

  void _fermeState();

  void _ouvertureState();

  void _fermetureState();


  // actions

  void _ouvrir();

  void _fermer();


  void _mettreAJourEtat();
  



  // Aide

  long _angleEnSteps(float angle) const;
};