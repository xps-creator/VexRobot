#pragma once
/*
 * Régulateur PID indépendant de PROS, pour pouvoir le tester sur ordinateur.
 * Utilisé pour avancer d'une distance précise et pour tourner d'un angle précis.
 */
#include <algorithm>
#include <cmath>

class Pid {
public:
    Pid(double kp, double ki, double kd, double sortieMax = 127.0, double zoneIntegrale = 10.0)
        : kp_(kp), ki_(ki), kd_(kd), sortieMax_(sortieMax), zoneIntegrale_(zoneIntegrale) {}

    void reinitialiser() {
        integrale_ = 0.0;
        erreurPrecedente_ = 0.0;
        premierAppel_ = true;
    }

    /* dt en secondes. Renvoie une commande bornée entre plus et moins sortieMax. */
    double calculer(double consigne, double mesure, double dt) {
        const double erreur = consigne - mesure;

        /* Anti saturation : on n'intègre que près de la cible, et on remet à zéro
           si l'erreur change de signe (la cible vient d'être dépassée). */
        if (std::fabs(erreur) < zoneIntegrale_) {
            integrale_ += erreur * dt;
        } else {
            integrale_ = 0.0;
        }
        if (!premierAppel_ && std::signbit(erreur) != std::signbit(erreurPrecedente_)) {
            integrale_ = 0.0;
        }

        const double derivee = premierAppel_ || dt <= 0.0 ? 0.0 : (erreur - erreurPrecedente_) / dt;
        erreurPrecedente_ = erreur;
        premierAppel_ = false;

        const double sortie = kp_ * erreur + ki_ * integrale_ + kd_ * derivee;
        return std::clamp(sortie, -sortieMax_, sortieMax_);
    }

    double erreur() const { return erreurPrecedente_; }

private:
    double kp_, ki_, kd_;
    double sortieMax_;
    double zoneIntegrale_;
    double integrale_ = 0.0;
    double erreurPrecedente_ = 0.0;
    bool premierAppel_ = true;
};

/* Courbe exponentielle sur le joystick : plus de précision à faible vitesse. */
inline int courbeJoystick(int valeur, double intensite = 0.6, int zoneMorte = 6) {
    if (std::abs(valeur) < zoneMorte) {
        return 0;
    }
    const double x = valeur / 127.0;
    const double y = (1.0 - intensite) * x + intensite * x * x * x;
    return static_cast<int>(std::lround(y * 127.0));
}

/* Plus petit écart signé entre deux caps, en degrés, dans l'intervalle de moins 180 à 180. */
inline double ecartAngle(double cible, double actuel) {
    double e = std::fmod(cible - actuel + 540.0, 360.0) - 180.0;
    return e;
}
