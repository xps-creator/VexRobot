#include "main.h"
#include "robot.hpp"

#include <string>

/* Programme principal PROS : initialisation, mode autonome et pilotage. */

Robot robot;
pros::Controller manette(pros::E_CONTROLLER_MASTER);

/* Choix du programme autonome sur l'écran du cerveau, avec les boutons gauche et droit. */
enum class Autonome { GAUCHE, DROITE, COMPETENCES };
Autonome choix = Autonome::GAUCHE;

const char* nomAutonome(Autonome a) {
    switch (a) {
        case Autonome::GAUCHE: return "Coin gauche";
        case Autonome::DROITE: return "Coin droit";
        default: return "Epreuve competences";
    }
}

void afficherChoix() {
    pros::lcd::set_text(1, std::string("Autonome : ") + nomAutonome(choix));
}

void boutonGauche() {
    choix = choix == Autonome::GAUCHE ? Autonome::COMPETENCES
          : choix == Autonome::DROITE ? Autonome::GAUCHE : Autonome::DROITE;
    afficherChoix();
}

void boutonDroit() {
    choix = choix == Autonome::GAUCHE ? Autonome::DROITE
          : choix == Autonome::DROITE ? Autonome::COMPETENCES : Autonome::GAUCHE;
    afficherChoix();
}

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Calibration de la centrale inertielle");
    robot.calibrer();
    pros::lcd::set_text(0, "Robot pret");
    afficherChoix();
    pros::lcd::register_btn0_cb(boutonGauche);
    pros::lcd::register_btn2_cb(boutonDroit);
}

void disabled() {}

void competition_initialize() {}

/* Routine type : saisir un objet, le lever, le déposer et revenir. */
void autonomeCoin(bool cote_gauche) {
    robot.pince(false);
    robot.avancer(60);
    robot.pince(true);
    pros::delay(250);
    robot.brasVers(300);
    robot.tournerVers(cote_gauche ? 90.0 : 270.0);
    robot.avancer(45);
    robot.brasVers(120);
    robot.pince(false);
    pros::delay(200);
    robot.avancer(-30);
    robot.brasVers(0);
}

void autonomeCompetences() {
    for (int tour = 0; tour < 4; ++tour) {
        robot.pince(false);
        robot.avancer(50);
        robot.pince(true);
        robot.brasVers(300);
        robot.tournerVers(90.0 * ((tour + 1) % 4));
        robot.avancer(40);
        robot.pince(false);
        robot.brasVers(0);
    }
}

void autonomous() {
    switch (choix) {
        case Autonome::GAUCHE: autonomeCoin(true); break;
        case Autonome::DROITE: autonomeCoin(false); break;
        case Autonome::COMPETENCES: autonomeCompetences(); break;
    }
}

/*
 * Commandes :
 *   joystick gauche vertical : avancer et reculer
 *   joystick droit horizontal : tourner
 *   R1 et R2 : monter et descendre le bras
 *   L1 : ouvrir ou fermer la pince
 *   B (maintenu) : mode précision, vitesse divisée par deux
 */
void opcontrol() {
    int cycle = 0;
    while (true) {
        int avance = courbeJoystick(manette.get_analog(ANALOG_LEFT_Y));
        int rotation = courbeJoystick(manette.get_analog(ANALOG_RIGHT_X));
        if (manette.get_digital(DIGITAL_B)) {
            avance /= 2;
            rotation /= 2;
        }
        robot.piloter(avance, rotation);

        if (manette.get_digital(DIGITAL_R1)) {
            robot.bras(110);
        } else if (manette.get_digital(DIGITAL_R2)) {
            robot.bras(-110);
        } else {
            robot.bras(0);
        }

        if (manette.get_digital_new_press(DIGITAL_L1)) {
            robot.pince(!robot.pinceFermee());
            manette.rumble(".");
        }

        /* Rafraîchir l'écran de la manette toutes les 250 ms (il est lent). */
        if (++cycle % 25 == 0) {
            manette.print(0, 0, "Cap %3.0f  Pince %s", robot.cap(), robot.pinceFermee() ? "F" : "O");
        }
        pros::delay(10);
    }
}
