#include "robot.hpp"

#include <cmath>

Robot::Robot()
    : gauche_({ports::GAUCHE_AVANT, ports::GAUCHE_ARRIERE}),
      droite_({ports::DROITE_AVANT, ports::DROITE_ARRIERE}),
      bras_(ports::BRAS),
      imu_(ports::IMU),
      pince_(ports::PINCE) {}

void Robot::calibrer() {
    imu_.reset(true); /* bloquant : environ deux secondes, robot immobile */
    gauche_.tare_position();
    droite_.tare_position();
    bras_.tare_position();
    bras_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    gauche_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    droite_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

/* Pilotage arcade : un joystick pour avancer, l'autre pour tourner. */
void Robot::piloter(int avance, int rotation) {
    int g = avance + rotation;
    int d = avance - rotation;
    /* Si une commande dépasse 127, on réduit les deux pour garder la trajectoire. */
    const int maxi = std::max(std::abs(g), std::abs(d));
    if (maxi > 127) {
        g = g * 127 / maxi;
        d = d * 127 / maxi;
    }
    gauche_.move(g);
    droite_.move(d);
}

double Robot::cap() {
    return imu_.get_heading();
}

double Robot::distanceParcourueCm() {
    const double degres = (gauche_.get_position() + droite_.get_position()) / 2.0;
    return degres * geometrie::CM_PAR_DEGRE;
}

bool Robot::avancer(double distanceCm, int vitesseMax, int delaiMaxMs) {
    Pid pidDistance(4.0, 0.02, 0.35, vitesseMax, 8.0);
    Pid pidCap(1.8, 0.0, 0.1, 40.0);
    const double capInitial = cap();
    gauche_.tare_position();
    droite_.tare_position();

    const uint32_t debut = pros::millis();
    int stable = 0;
    while (pros::millis() - debut < static_cast<uint32_t>(delaiMaxMs)) {
        const double parcouru = distanceParcourueCm();
        const int vitesse = static_cast<int>(pidDistance.calculer(distanceCm, parcouru, 0.01));
        /* Correction de cap : garde le robot droit pendant la ligne droite. */
        const int correction = static_cast<int>(pidCap.calculer(0.0, -ecartAngle(capInitial, cap()), 0.01));
        gauche_.move(vitesse + correction);
        droite_.move(vitesse - correction);

        stable = std::fabs(distanceCm - parcouru) < 1.0 ? stable + 1 : 0;
        if (stable >= 10) { /* 100 ms dans la tolérance */
            piloter(0, 0);
            return true;
        }
        pros::delay(10);
    }
    piloter(0, 0);
    return false;
}

bool Robot::tournerVers(double capDegres, int vitesseMax, int delaiMaxMs) {
    Pid pidRotation(2.2, 0.03, 0.18, vitesseMax, 15.0);
    const uint32_t debut = pros::millis();
    int stable = 0;
    while (pros::millis() - debut < static_cast<uint32_t>(delaiMaxMs)) {
        const double erreur = ecartAngle(capDegres, cap());
        /* On donne au PID une mesure relative pour qu'il tourne toujours par le plus court chemin. */
        const int commande = static_cast<int>(pidRotation.calculer(erreur, 0.0, 0.01));
        piloter(0, commande);
        stable = std::fabs(erreur) < 1.5 ? stable + 1 : 0;
        if (stable >= 10) {
            piloter(0, 0);
            return true;
        }
        pros::delay(10);
    }
    piloter(0, 0);
    return false;
}

void Robot::bras(int vitesse) {
    if (vitesse == 0) {
        bras_.brake(); /* mode HOLD : le bras reste en place */
    } else {
        bras_.move(vitesse);
    }
}

void Robot::brasVers(double angleDegres, int vitesseMax, int delaiMaxMs) {
    Pid pidBras(1.2, 0.0, 0.05, vitesseMax);
    const uint32_t debut = pros::millis();
    while (pros::millis() - debut < static_cast<uint32_t>(delaiMaxMs)) {
        const double position = bras_.get_position();
        if (std::fabs(angleDegres - position) < 3.0) {
            break;
        }
        bras_.move(static_cast<int>(pidBras.calculer(angleDegres, position, 0.01)));
        pros::delay(10);
    }
    bras_.brake();
}

void Robot::pince(bool fermee) {
    pinceFermee_ = fermee;
    pince_.set_value(fermee);
}
