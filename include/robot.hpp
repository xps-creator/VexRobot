#pragma once
#include "main.h"
#include "pid.hpp"

/* Câblage du robot (numéros de ports V5). Un port négatif inverse le moteur. */
namespace ports {
constexpr int8_t GAUCHE_AVANT = 1;
constexpr int8_t GAUCHE_ARRIERE = 2;
constexpr int8_t DROITE_AVANT = -3;
constexpr int8_t DROITE_ARRIERE = -4;
constexpr int8_t BRAS = 8;
constexpr int8_t IMU = 10;
constexpr char PINCE = 'A';
}

/* Dimensions utiles pour convertir les degrés moteur en centimètres. */
namespace geometrie {
constexpr double DIAMETRE_ROUE_CM = 10.16; /* roues de 4 pouces */
constexpr double RAPPORT = 1.0;            /* transmission directe */
constexpr double CM_PAR_DEGRE = 3.14159265 * DIAMETRE_ROUE_CM / 360.0 * RAPPORT;
}

class Robot {
public:
    Robot();

    void calibrer();
    void piloter(int avance, int rotation);

    /* Déplacements autonomes, bloquants, avec délai maximal de sécurité. */
    bool avancer(double distanceCm, int vitesseMax = 100, int delaiMaxMs = 3000);
    bool tournerVers(double capDegres, int vitesseMax = 90, int delaiMaxMs = 2000);

    void bras(int vitesse);
    void brasVers(double angleDegres, int vitesseMax = 100, int delaiMaxMs = 1500);
    void pince(bool fermee);
    bool pinceFermee() const { return pinceFermee_; }

    double cap();
    double distanceParcourueCm();

private:
    pros::MotorGroup gauche_;
    pros::MotorGroup droite_;
    pros::Motor bras_;
    pros::Imu imu_;
    pros::adi::DigitalOut pince_;
    bool pinceFermee_ = false;
};
