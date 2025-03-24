#include "EMAFilter.h"
#include <math.h>

// Constructeur
EMAFilter::EMAFilter(float alpha) {
    this->alpha = alpha;
    this->initialized = false;
}

// Mise à jour du filtre avec une nouvelle mesure
float EMAFilter::update(float newValue) {
    if (!initialized) {
        filteredValue = newValue; // Initialisation avec la première valeur
        initialized = true;
    } else {
        filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
    }

    // Arrondi à une seule décimale
    filteredValue = round(filteredValue * 10) / 10.0;

    return filteredValue;
}

// Retourne la dernière valeur filtrée
float EMAFilter::getFilteredValue() {
    return filteredValue;
}
