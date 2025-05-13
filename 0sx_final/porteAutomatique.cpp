#include "PorteAutomatique.h"
#include <Arduino.h>

PorteAutomatique::PorteAutomatique(int p1, int p2, int p3, int p4, float& distancePtr)
  : _stepper(AccelStepper::HALF4WIRE, p1, p3, p2, p4), _distance(distancePtr) {
  _stepper.setMaxSpeed(1000);
  _stepper.setAcceleration(500);
  _stepper.setCurrentPosition(_angleEnSteps(_angleFerme));
  _stepper.moveTo(_angleEnSteps(_angleFerme));
  _stepper.enableOutputs();
}

void PorteAutomatique::update() {
  _currentTime = millis();
  _mettreAJourEtat();

  switch (_etat) {
    case FERMEE: _fermeState(); break;
    case OUVERTE: _ouvertState(); break;
    case EN_OUVERTURE: _ouvertureState(); break;
    case EN_FERMETURE: _fermetureState(); break;
  }

  _stepper.run();
}

void PorteAutomatique::setAngleOuvert(float angle) {
  _angleOuvert = angle;
}

void PorteAutomatique::setAngleFerme(float angle) {
  _angleFerme = angle;
}

void PorteAutomatique::setPasParTour(int steps) {
  _stepsPerRev = steps;
}

void PorteAutomatique::setDistanceOuverture(float distance) {
  _distanceOuverture = distance;
}

void PorteAutomatique::setDistanceFermeture(float distance) {
  _distanceFermeture = distance;
}

const char* PorteAutomatique::getEtatTexte() const {
  switch (_etat) {
    case FERMEE: return "Fermee";
    case OUVERTE: return "Ouverte";
    case EN_OUVERTURE: return "Ouverture...";
    case EN_FERMETURE: return "Fermeture...";
    default: return "Inconnu";
  }
}

float PorteAutomatique::getAngle() const {
  long steps = _stepper.currentPosition();
  float angle = (float)steps * 360.0 / _stepsPerRev;
  return angle;
}

long PorteAutomatique::_angleEnSteps(float angle) const {
  return (long)(angle * _stepsPerRev / 360.0);
}

void PorteAutomatique::_mettreAJourEtat() {
  if (_etat == FERMEE && _distance < _distanceOuverture && _distance > 0) {
    _etat = EN_OUVERTURE;
    _ouvrir();
  } else if (_etat == OUVERTE && _distance > _distanceFermeture) {
    _etat = EN_FERMETURE;
    _fermer();
  }
}

void PorteAutomatique::_ouvrir() {
  _stepper.enableOutputs();
  _stepper.moveTo(_angleEnSteps(_angleOuvert));
}

void PorteAutomatique::_fermer() {
  _stepper.enableOutputs();
  _stepper.moveTo(_angleEnSteps(_angleFerme));
}
void PorteAutomatique::enableOutput() {
  _stepper.enableOutputs();
}

void PorteAutomatique::disableOutput() {
  _stepper.disableOutputs();
}

void PorteAutomatique::_ouvertState() {
  _stepper.disableOutputs();
}

void PorteAutomatique::_fermeState() {
  _stepper.disableOutputs();
}

void PorteAutomatique::_ouvertureState() {
  if (!_stepper.isRunning()) {
    _etat = OUVERTE;
  }
}

void PorteAutomatique::_fermetureState() {
  if (!_stepper.isRunning()) {
    _etat = FERMEE;
  }
}
