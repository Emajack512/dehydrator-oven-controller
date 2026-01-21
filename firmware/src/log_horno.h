#pragma once
#include <TFT_eSPI.h>

// Dimensiones de la imagen
#define LOGO_HORNO_W  480
#define LOGO_HORNO_H  320

// Solo exportamos la función, el array queda escondido en Imagen.cpp
void drawLogoHorno(TFT_eSPI &tft, int x, int y);