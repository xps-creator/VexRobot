/* Tests du PID et des fonctions utilitaires, sur ordinateur :
   g++ -std=c++17 -Iinclude tests/test_pid.cpp -o test_pid && ./test_pid */
#include "pid.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

/* Modèle simple d'un robot : la vitesse suit la commande avec du retard. */
static double simulerDeplacement(double cible, int pas) {
    Pid pid(4.0, 0.02, 0.35, 100.0, 8.0);
    double position = 0.0, vitesse = 0.0;
    const double dt = 0.01;
    for (int i = 0; i < pas; ++i) {
        const double commande = pid.calculer(cible, position, dt);
        vitesse += (commande * 0.8 - vitesse) * 0.08; /* inertie */
        position += vitesse * dt;
    }
    return position;
}

int main() {
    /* La sortie est bornée */
    Pid p(10.0, 0.0, 0.0, 127.0);
    assert(p.calculer(1000.0, 0.0, 0.01) == 127.0);
    assert(p.calculer(-1000.0, 0.0, 0.01) == -127.0);

    /* Pas de dérivée au premier appel (pas de pic au démarrage) */
    Pid d(0.0, 0.0, 5.0);
    assert(d.calculer(50.0, 0.0, 0.01) == 0.0);

    /* Le PID amène le robot simulé à moins d'un centimètre de la cible */
    const double finale = simulerDeplacement(60.0, 400);
    std::printf("position finale simulée : %.2f cm pour 60 cm\n", finale);
    assert(std::fabs(finale - 60.0) < 1.0);

    /* Écart d'angle : toujours le plus court chemin */
    assert(std::fabs(ecartAngle(10.0, 350.0) - 20.0) < 1e-9);
    assert(std::fabs(ecartAngle(350.0, 10.0) + 20.0) < 1e-9);
    assert(std::fabs(ecartAngle(90.0, 90.0)) < 1e-9);

    /* Courbe du joystick : zone morte, symétrie, bornes */
    assert(courbeJoystick(3) == 0);
    assert(courbeJoystick(127) == 127);
    assert(courbeJoystick(-127) == -127);
    assert(courbeJoystick(64) == -courbeJoystick(-64));
    assert(courbeJoystick(64) < 64); /* plus doux à mi course */

    std::printf("tous les tests sont passés\n");
    return 0;
}
