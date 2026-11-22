# Robot VEX V5

Programme d'un robot VEX V5 : conduite à la manette, bras motorisé, pince pneumatique et routines autonomes. Écrit en C++ avec PROS, l'environnement open source pour VEX V5.

## Ce que fait le programme

**Pilotage** (`opcontrol`)
* Conduite arcade : joystick gauche pour avancer, joystick droit pour tourner
* Courbe exponentielle et zone morte sur les joysticks, pour être précis à faible vitesse
* Mode précision (bouton B maintenu) qui divise la vitesse par deux
* Bras sur R1 et R2, maintenu en position par le frein moteur
* Pince pneumatique sur L1, avec retour par vibration de la manette
* Cap et état de la pince affichés sur l'écran de la manette

**Autonome** (`autonomous`)
* Trois routines au choix (coin gauche, coin droit, épreuve compétences), sélectionnées avant le match avec les boutons de l'écran du cerveau
* Déplacements en ligne droite régulés par PID sur les encodeurs des moteurs, avec correction de cap par la centrale inertielle
* Rotations régulées par PID sur la centrale inertielle, toujours par le plus court chemin
* Délais maximaux sur chaque mouvement pour ne jamais rester bloqué pendant un match

## Le régulateur PID

`include/pid.hpp` ne dépend pas de PROS et se teste sur ordinateur. Il comprend :
* une sortie bornée à la puissance maximale des moteurs ;
* un anti saturation de l'intégrale (intégration seulement près de la cible, remise à zéro quand la cible est dépassée) ;
* pas de dérivée au premier appel, pour éviter un à coup au démarrage.

```bash
g++ -std=c++17 -Iinclude tests/test_pid.cpp -o test_pid && ./test_pid
# position finale simulée : 60.00 cm pour 60 cm
# tous les tests sont passés
```

Le test simule un robot avec de l'inertie et vérifie qu'il s'arrête à moins d'un centimètre de la cible.

## Câblage

| Élément | Port |
|:---|:---|
| Moteurs gauche (avant, arrière) | 1, 2 |
| Moteurs droite (avant, arrière, inversés) | 3, 4 |
| Bras | 8 |
| Centrale inertielle | 10 |
| Pince pneumatique | ADI A |

Les ports se changent dans `include/robot.hpp`.

## Compiler et envoyer sur le robot

Avec PROS 4 installé (extension VS Code ou ligne de commande) :

```bash
pros conductor new-project robot-vex
# copier src/ et include/ de ce dépôt dans le projet
pros make
pros upload
```

## Structure

```
include/pid.hpp     régulateur PID, courbe joystick, écart d'angle
include/robot.hpp   câblage et interface de la classe Robot
src/robot.cpp       conduite, déplacements autonomes, bras, pince
src/main.cpp        initialisation, choix de l'autonome, pilotage
tests/test_pid.cpp  tests sur ordinateur
```

## Auteur

Nizar, élève ingénieur en Data Science et IA à CESI.
[LinkedIn](https://www.linkedin.com/in/nizar-outski-549941383/)
