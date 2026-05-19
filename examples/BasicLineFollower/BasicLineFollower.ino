#include <PID_5_Sensors.h>

// Exemple minimal d'utilisation de PID_5_Sensors
// Ce fichier montre comment intégrer la librairie dans un projet
// Les lectures de capteurs et commandes moteurs sont à adapter
// selon ton hardware (MCP3208, DRV8833, etc.)

PID_5_Sensors pid;

void setup()
{
    Serial.begin(115200);

    // 1 — Régler les paramètres PID
    pid.Kp        = 30.0f;
    pid.Kd        = 10.0f;
    pid.baseSpeed = 55;
    pid.maxSpeed  = 110;

    // 2 — Indiquer si on suit une ligne noire ou blanche
    pid.setWhiteLine(false);

    // 3 — Après calibration, injecter les seuils capteur par capteur
    //     pid.setThreshold(index, seuil_milieu, seuil_fond);
    //     Exemple si ton seuil calculé est 2.1V et fond à 2.8V :
    pid.setThreshold(0, 2.1f, 2.8f);
    pid.setThreshold(1, 2.0f, 2.7f);
    pid.setThreshold(2, 2.2f, 2.9f);
    pid.setThreshold(3, 2.1f, 2.8f);
    pid.setThreshold(4, 2.0f, 2.7f);

    pid.resetPID();
}

void loop()
{
    // 4 — Lire les 5 tensions (ici simulées, à remplacer par ton ADC)
    float raw[NB_SENSORS];
    for (int i = 0; i < NB_SENSORS; i++)
        raw[i] = 1.5f; // remplacer par : (adc.read(i) / 4095.0f) * 3.3f

    // 5 — Obtenir vitesse linéaire et angulaire depuis tes encodeurs (cm/s, rad/s)
    float vLin  = 10.0f; // remplacer par ta fonction lireVitesses()
    float omega =  0.0f;

    // 6 — Calculer la position centroïde
    float position = pid.centroid(raw, vLin, omega);

    // 7 — Détections utiles
    if (pid.isLost(raw))     { /* ligne perdue → arrêt */ return; }
    if (pid.isCrossing(raw)) { /* intersection détectée */ }

    // 8 — Calculer dt
    static unsigned long tPrev = micros();
    unsigned long tNow = micros();
    float dt = (tNow - tPrev) / 1e6f;
    tPrev = tNow;

    // 9 — Appliquer les vitesses moteurs
    int vG = pid.computeLeft(position, dt);
    int vD = pid.computeRight(position, dt);

    // moteurGauche(vG);
    // moteurDroit(vD);

    Serial.printf("pos=%.2f  vG=%d  vD=%d\n", position, vG, vD);
}
