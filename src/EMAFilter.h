#ifndef EMAFILTER_H
#define EMAFILTER_H

class EMAFilter {
  private:
    float alpha;  // Coefficient de lissage
    float filteredValue;
    bool initialized;

  public:
    EMAFilter(float alpha);   // Constructeur
    float update(float newValue);  // Met à jour la valeur filtrée
    float getFilteredValue();  // Retourne la dernière valeur filtrée
};

#endif
