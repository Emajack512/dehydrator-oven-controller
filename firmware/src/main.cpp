#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "log_horno.h"
#define BLYNK_TEMPLATE_ID "TMPL2UElM6kZk"
#define BLYNK_TEMPLATE_NAME "DRIVER TEMPERATURA"
#define BLYNK_AUTH_TOKEN "5_JM0xd365JNqcD-k6dGpot8jtnAs0qJ"
#define BLYNK_PRINT Serial   // para ver logs de Blynk por Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ===== WiFi / Blynk =====
char ssid[] = "moto84";
char pass[] = "123456798";

BlynkTimer blynkTimer;

// Variables que vamos a compartir entre todo
float tempActual = 0.0f;
float humActual  = 0.0f;   // si no querés humedad en la nube, igual la usamos en la TFT
float tempSet    = 60.0f;  // se actualizará desde Blynk

TFT_eSPI tft = TFT_eSPI();   // Configurado en User_Setup de TFT_eSPI

// ===== Colores =====
#define COLOR_BG          TFT_BLACK
#define COLOR_ARC_BG      TFT_DARKGREY
#define COLOR_TEMP        TFT_RED
#define COLOR_TEMP_SET    TFT_ORANGE
#define COLOR_HUM         TFT_CYAN

#define COLOR_GRAPH_AX    TFT_WHITE
#define COLOR_GRID_MAJOR  TFT_LIGHTGREY
#define COLOR_GRID_MINOR  TFT_DARKGREY
#define COLOR_GRAPH_TEMP  TFT_RED
#define COLOR_GRAPH_HUM   TFT_CYAN

Adafruit_SHT31 sht31 = Adafruit_SHT31();
#define SHT31_SDA 21
#define SHT31_SCL 22

volatile uint32_t lastEdgeMicros = 0;     // último flanco (alto o bajo)
volatile uint32_t highTimeMicros = 0;     // duración último nivel ALTO
volatile uint32_t lowTimeMicros  = 0;     // duración último nivel BAJO
volatile bool     zcFlag         = false; // avisa al loop que hay dato nuevo

// ===== Control de potencia con TRIAC (burst-fire) =====
#define ZC_PIN      35   // entrada de cruce por cero
#define TRIAC_PIN   18   // SALIDA hacia MOC3063 (ajustá al pin que uses)

// Ventana de control: cantidad de semiperíodos que miramos
// 20 semiperíodos ~ 0.2 s (a 50 Hz) -> respuesta suave, ideal para horno
const uint16_t WINDOW_HALF_CYCLES = 20;

// Estas las usa la ISR
volatile uint16_t halfCyclePos    = 0;  // en qué semiperíodo de la ventana estoy
volatile uint16_t dutyHalfCycles  = 0;  // cuántos semiperíodos deben estar ON en la ventana

// Salida del PID en 0–1 (0 a 100% de potencia)
float pidOutput = 0.0f;

// ===== Config de gráficas =====
const int   GRAPH_MARGIN_LEFT = 40;          // espacio para etiquetas Y
const int   GRAPH_X  = GRAPH_MARGIN_LEFT;
const int   GRAPH_W  = 430;

const int   GRAPH_H  = 80;                  // altura de cada gráfica
const int   GRAPH1_Y = 60;                  // Temp (un poco más abajo)
const int   GRAPH2_Y = GRAPH1_Y + GRAPH_H + 15;  // Humedad

const int   GRAPH_POINTS       = 220;       // muestras visibles
const float SAMPLE_PERIOD_SEC  = 0.3f;      // tiempo entre muestras
const float GRID_TIME_STEP_SEC = 10.0f;     // división vertical cada 10 s
const float BAND_WIDTH         = 10.0f;     // ancho de banda (0–10,10–20,...)

// Historial
float tempHistory[GRAPH_POINTS];
float humHistory[GRAPH_POINTS];
int   histPos  = 0;
bool  histFull = false;

// Rangos dinámicos actuales de las escalas
float tempMinDyn = 0.0f, tempMaxDyn = 120.0f;
float humMinDyn  = 0.0f, humMaxDyn  = 100.0f;

// ================== HISTORIAL ==================
void addSample(float temp, float hum) {
  tempHistory[histPos] = temp;
  humHistory[histPos]  = hum;

  histPos++;
  if (histPos >= GRAPH_POINTS) {
    histPos = 0;
    histFull = true;
  }
}

int mapToGraphY(float value, float vmin, float vmax, int gy, int gh) {
  if (value < vmin) value = vmin;
  if (value > vmax) value = vmax;
  float frac = (value - vmin) / (float)(vmax - vmin); // 0..1
  int y = gy + gh - 1 - (int)(frac * (gh - 1));
  return y;
}

// ================== GRILLAS / EJES ==================
void drawGridAndAxes(int gx, int gy, int gw, int gh,
                     const char* title,
                     float vmin, float vmax, float yStep)
{
  // marco
  tft.drawRect(gx, gy, gw, gh, COLOR_GRAPH_AX);

  // título (arriba, alineado a la izquierda)
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString(title, gx, gy - 14, 2);

  // ticks y líneas horizontales
  int nTicks = (int)((vmax - vmin) / yStep);
  char buf[16];

  for (int i = 0; i <= nTicks; i++) {
    float val = vmin + i * yStep;
    int y = mapToGraphY(val, vmin, vmax, gy, gh);

    // línea de grilla horizontal
    uint16_t col = (i == 0 || i == nTicks) ? COLOR_GRID_MAJOR : COLOR_GRID_MINOR;
    tft.drawLine(gx, y, gx + gw - 1, y, col);

    // pequeña marca sobre eje Y
    tft.drawLine(gx - 3, y, gx, y, COLOR_GRAPH_AX);

    // texto del valor (a la izquierda, valores absolutos)
    dtostrf(val, 0, 0, buf);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, gx - 5, y - 6, 2);
  }
}

// ================== DIBUJO DE GRÁFICAS ==================
void drawGraphs() {
  int graphsTop    = GRAPH1_Y - 20;
  int graphsBottom = GRAPH2_Y + GRAPH_H;
  int graphsHeight = graphsBottom - graphsTop;

  // limpiar zona de gráficas (sin tocar el título)
  tft.fillRect(0, graphsTop, 480, graphsHeight + 5, COLOR_BG);

  // grillas + ejes de ambas (usando rangos dinámicos)
  drawGridAndAxes(GRAPH_X, GRAPH1_Y, GRAPH_W, GRAPH_H,
                  "Temp [C]", tempMinDyn, tempMaxDyn, 2.0);
  drawGridAndAxes(GRAPH_X, GRAPH2_Y, GRAPH_W, GRAPH_H,
                  "Humedad [%]", humMinDyn, humMaxDyn, 2.0);

  // ===== Grilla vertical por tiempo (segundos) =====
  float totalSec = GRAPH_POINTS * SAMPLE_PERIOD_SEC;
  int xStart = GRAPH_X;
  int xEnd   = GRAPH_X + GRAPH_W - 1;
  int yTop   = GRAPH1_Y;
  int yBot   = GRAPH2_Y + GRAPH_H - 1;

  for (float s = 0; s <= totalSec + 0.01f; s += GRID_TIME_STEP_SEC) {
    float frac = s / totalSec;              // 0..1
    int x = xStart + (int)(frac * (GRAPH_W - 1));
    if (x < xStart) x = xStart;
    if (x > xEnd)   x = xEnd;
    tft.drawLine(x, yTop, x, yBot, COLOR_GRID_MINOR);
  }

  // etiqueta de tiempo en la gráfica de abajo
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COLOR_GRAPH_AX, COLOR_BG);
  tft.drawString("Tiempo [s]",
                 GRAPH_X + GRAPH_W / 2,
                 GRAPH2_Y + GRAPH_H + 6, 2);

  // ===== curvas =====
  int points = histFull ? GRAPH_POINTS : histPos;
  if (points < 2) return;

  int startIndex = histFull ? histPos : 0;

  for (int i = 0; i < points - 1; i++) {
    int idx1 = (startIndex + i)     % GRAPH_POINTS;
    int idx2 = (startIndex + i + 1) % GRAPH_POINTS;

    int x1 = GRAPH_X + (i * (GRAPH_W - 1)) / (points - 1);
    int x2 = GRAPH_X + ((i + 1) * (GRAPH_W - 1)) / (points - 1);

    // temperatura
    int y1t = mapToGraphY(tempHistory[idx1], tempMinDyn, tempMaxDyn, GRAPH1_Y, GRAPH_H);
    int y2t = mapToGraphY(tempHistory[idx2], tempMinDyn, tempMaxDyn, GRAPH1_Y, GRAPH_H);
    tft.drawLine(x1, y1t, x2, y2t, COLOR_GRAPH_TEMP);

    // humedad
    int y1h = mapToGraphY(humHistory[idx1], humMinDyn, humMaxDyn, GRAPH2_Y, GRAPH_H);
    int y2h = mapToGraphY(humHistory[idx2], humMinDyn, humMaxDyn, GRAPH2_Y, GRAPH_H);
    tft.drawLine(x1, y1h, x2, y2h, COLOR_GRAPH_HUM);
  }
}

// ================== GAUGES ==================
void drawGauge(int cx, int cy, int radius, int thickness,
               float value, float vmin, float vmax,
               const char *label, uint16_t color)
{
  if (value < vmin) value = vmin;
  if (value > vmax) value = vmax;

  float frac = (value - vmin) / (float)(vmax - vmin);   // 0..1

  int startDeg = 150;
  int endDeg   = 30;

  // fondo del arco
  for (int a = startDeg; a >= endDeg; a--) {
    float rad = a * DEG_TO_RAD;
    int xOuter = cx + cos(rad) * radius;
    int yOuter = cy - sin(rad) * radius;
    int xInner = cx + cos(rad) * (radius - thickness);
    int yInner = cy - sin(rad) * (radius - thickness);
    tft.drawLine(xInner, yInner, xOuter, yOuter, COLOR_ARC_BG);
  }

  // arco de valor
  int valueDeg = startDeg - (int)((startDeg - endDeg) * frac);

  for (int a = startDeg; a >= valueDeg; a--) {
    float rad = a * DEG_TO_RAD;
    int xOuter = cx + cos(rad) * radius;
    int yOuter = cy - sin(rad) * radius;
    int xInner = cx + cos(rad) * (radius - thickness);
    int yInner = cy - sin(rad) * (radius - thickness);
    tft.drawLine(xInner, yInner, xOuter, yOuter, color);
  }

  // número dentro del arco
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);

  char buf[16];
  dtostrf(value, 0, 1, buf);
  tft.drawString(buf, cx, cy + 2, 4);

  // etiqueta justo debajo del número
  tft.drawString(label, cx, cy + 25, 2);
}

void drawGauges(float tempAct, float tempSet, float humAct)
{
  // zona de gauges en la parte baja
  tft.fillRect(0, 240, 480, 80, COLOR_BG);

  int cy     = 280;   // posición vertical (abajo)
  int radius = 35;    // tamaño de los medidores
  int thick  = 10;

  // Tres gauges: Temp actual, Temp set, Humedad actual
  drawGauge( 80, cy, radius, thick, tempAct, 0, 120, "Temp (C)",   COLOR_TEMP);
  drawGauge(240, cy, radius, thick, tempSet, 0, 120, "Set Temp",   COLOR_TEMP_SET);
  drawGauge(400, cy, radius, thick, humAct,  0, 100, "Humedad %",  COLOR_HUM);
}



// ========================BLYNK IOT=====================
// Slider de setpoint de temperatura en V1
// Slider de setpoint de temperatura en V2 (app -> ESP32)
BLYNK_WRITE(V2) {
  tempSet = param.asFloat();
}

// Al conectar, traer el setpoint guardado en el servidor
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V2);
}

void sendToBlynk() {
  Blynk.virtualWrite(V0, tempActual);  // Temperatura actual (°C)
  Blynk.virtualWrite(V1, humActual);   // Humedad actual (%)
}

// ===== PID de temperatura (discreto) =====
float Kp = 1.5f;   // ajustá a gusto después
float Ki = 0.05f;
float Kd = 0.0f;

void updatePID() {
  static float integral   = 0.0f;
  static float prevError  = 0.0f;
  static uint32_t lastMs  = 0;

  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;   // en segundos
  if (dt <= 0.0f) {
    lastMs = now;
    return;
  }
  lastMs = now;

  float error = tempSet - tempActual;    // °C
  float derivative = (error - prevError) / dt;
  prevError = error;

  // --- integral candidata ---
  float integralCandidate = integral + error * dt;

  // salida SIN saturar usando la integral candidata
  float u_unsat = Kp * error + Ki * integralCandidate + Kd * derivative;

  // --- saturación 0–1 ---
  float u = u_unsat;
  if (u > 1.0f) u = 1.0f;
  if (u < 0.0f) u = 0.0f;

  // --- anti-windup simple ---
  // Si estamos saturados y el u_unsat quiere seguir en la misma dirección,
  // NO actualizamos la integral (evitamos que se dispare).
  bool satAlta = (u >= 1.0f && u_unsat > 1.0f);
  bool satBaja = (u <= 0.0f && u_unsat < 0.0f);

  if (!satAlta && !satBaja) {
    integral = integralCandidate;
  }

  pidOutput = u;

  // --- u (0–1) -> duty en semiperíodos ---
  uint16_t duty = 0;
  if (u > 0.0f) {
    duty = (uint16_t)(u * WINDOW_HALF_CYCLES + 0.5f);
  }

  noInterrupts();
  dutyHalfCycles = duty;
  interrupts();

  // debug
  Serial.print("PID: error = ");
  Serial.print(error);
  Serial.print("  u = ");
  Serial.print(u);
  Serial.print("  dutyHalfCycles = ");
  Serial.println(dutyHalfCycles);
}


void IRAM_ATTR onZeroCross() {
  uint32_t now = micros();
  bool level = digitalRead(ZC_PIN);  // leemos si está en ALTO o BAJO

  // --- Medición de tiempos alto/bajo ---
  if (lastEdgeMicros != 0) {   // evitamos la primera medida basura
    if (level) {
      // Pasamos de BAJO -> ALTO: terminó un nivel BAJO
      lowTimeMicros = now - lastEdgeMicros;
    } else {
      // Pasamos de ALTO -> BAJO: terminó un nivel ALTO
      highTimeMicros = now - lastEdgeMicros;
    }
    zcFlag = true;  // avisamos al loop que hay nuevo dato
  }
  lastEdgeMicros = now;

  // --- Control de potencia por ventana (burst-fire) ---
  if (halfCyclePos >= WINDOW_HALF_CYCLES) {
    halfCyclePos = 0;   // reiniciamos ventana
  }

  bool fire = (halfCyclePos < dutyHalfCycles);
  halfCyclePos++;

  if (fire) {
    digitalWrite(TRIAC_PIN, HIGH);
  } else {
    digitalWrite(TRIAC_PIN, LOW);
  }
}


// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(9600);
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);


  tft.setSwapBytes(true);
  drawLogoHorno(tft, 0, 0);   // o centrada si querés

  delay(5000);

  tft.fillScreen(COLOR_BG);

  // ----- HEADER PARA EL TITULO -----
  const int headerH = 32;               // altura de la barra superior
  tft.fillRect(0, 0, 480, headerH, COLOR_BG);

  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextDatum(MC_DATUM);           // centrado X e Y
  tft.drawString("Panel de Control", 240, headerH / 2 + 1, 4); // fuente 4

  // ========== INICIO I2C + SHT31 ==========
  Wire.begin(SHT31_SDA, SHT31_SCL);

  if (!sht31.begin(0x44)) {   // 0x44 es la dirección típica del SHT31
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.drawString("ERROR SHT31", 5, headerH + 5, 2);
    while (1) {
      delay(100);             // se queda trabado si no lo encuentra
    }
  } else {
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_GREEN, COLOR_BG);
    tft.drawString("SHT31 OK", 5, headerH + 5, 2);
  }

    // ===== INICIO TRIAC + cruce por cero =====
  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);

  pinMode(ZC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ZC_PIN), onZeroCross, CHANGE);


  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  blynkTimer.setInterval(5000L, sendToBlynk);   // ya lo tenías
  blynkTimer.setInterval(1000L, updatePID);     // PID cada 1 s

}

void loop() {
  
  //Inicio Blynk
  Blynk.run();
  blynkTimer.run();

  //Leer temperaturas con el SHT31
  tempActual = sht31.readTemperature(); // °C
  humActual  = sht31.readHumidity();    // %

 //    Si falla la lectura, no actualizamos la pantalla en este ciclo
 if (isnan(tempActual) || isnan(humActual)) {

    //Mensaje de error 
    tft.setTextDatum(TL_DATUM);
   tft.setTextColor(TFT_RED, COLOR_BG);
   tft.drawString("Lectura invalida SHT31", 5, 50, 2);
   delay((int)(SAMPLE_PERIOD_SEC * 1000));
   return;
  }


  // ===== actualizar escalas dinámicas por bandas de 10 =====
  float bandT = floor(tempActual / BAND_WIDTH);
  tempMinDyn = bandT * BAND_WIDTH;
  tempMaxDyn = tempMinDyn + BAND_WIDTH;

  float bandH = floor(humActual / BAND_WIDTH);
  humMinDyn = bandH * BAND_WIDTH;
  humMaxDyn = humMinDyn + BAND_WIDTH;

  // 1) guardar muestra
  addSample(tempActual, humActual);

  // 2) redibujar gráficas con nuevas escalas
  drawGraphs();

  // 3) redibujar gauges (solo 3: tempAct, tempSet, humAct)
  drawGauges(tempActual, tempSet, humActual);
  
  // 4) Periodo de muestreo 
  delay((int)(SAMPLE_PERIOD_SEC * 1000));

  if (zcFlag) {
  noInterrupts();
  zcFlag = false;
  uint32_t hi = highTimeMicros;
  uint32_t lo = lowTimeMicros;
  interrupts();

  // Evitamos divisiones raras si todavía no se midieron los dos
  if (hi > 0 && lo > 0) {
    float T_half_hi_us = (float)hi;            // nivel ALTO
    float T_half_lo_us = (float)lo;            // nivel BAJO
    float T_full_ms    = (hi + lo) / 1000.0f;  // período completo en ms
    float fline        = 1e6f / (hi + lo);     // f = 1/T, con T en us

    Serial.print("ALTO = ");
    Serial.print(T_half_hi_us);
    Serial.print(" us | BAJO = ");
    Serial.print(T_half_lo_us);
    Serial.print(" us | T = ");
    Serial.print(T_full_ms, 3);
    Serial.print(" ms | f = ");
    Serial.print(fline, 3);
    Serial.println(" Hz");
  }
}

  Serial.print("Temperatura: ");
  Serial.print(tempActual);   
  Serial.print(" | Humedad: ");
  Serial.println(humActual);
  Serial.print(" | Setpoint: ");
  Serial.println(tempSet);
}