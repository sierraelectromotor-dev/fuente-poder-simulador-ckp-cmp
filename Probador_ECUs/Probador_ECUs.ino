/*
 * ======================================================================================
 * PROBADOR DE ECUs AUTOMOTRICES - FUENTE DE LABORATORIO + GENERADOR CKP / CMP + SERIAL
 * ======================================================================================
 * Plataforma: Compatible con Arduino UNO (ATmega328P) y Arduino MEGA 2560
 * Display:    TFT 3.5" Paralelo 8-bit Shield directo (MCUFRIEND_kbv)
 * Touch:      Panel Táctil Resistivo 4 hilos con Calibración Interactiva en EEPROM
 *
 * MAPA DE PINES:
 * --------------------------------------------------------------------------------------
 *   - A5        : Salida analógica del sensor de corriente ACS712 (20A: 100mV/A) [100% LIBRE]
 *   - A4 (o A6+ en MEGA) : Entrada Divisor de Voltaje
 *   - D10       : Salida de Señal CKP (Cigüeñal - Crankshaft Position) por Timer1
 *   - D11       : Salida de Señal CMP (Árbol de Levas - Camshaft Position) por Timer1
 *   - D12       : Salida de Relé / MOSFET de Alimentación ECU (+12V Ignición / Main)
 *
 * CALIBRACIÓN TÁCTIL EN VIVO (GUARDADA EN EEPROM):
 * --------------------------------------------------------------------------------------
 *   - Mantén presionada la pantalla al encender/resetear el Arduino, O
 *   - Toca el botón "CAL TOUCH" en la esquina superior derecha, O
 *   - Envía el comando `CAL` desde el Monitor Serial (115200 bps).
 * ======================================================================================
 */

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

// ======================================================================================
// DRIVER INTEGRADO PARA PANEL TÁCTIL RESISTIVO 4 HILOS (COMPACTO)
// ======================================================================================
struct TSPoint {
  int16_t x, y, z;
};

uint8_t YP = A2;
uint8_t XM = A1;
uint8_t YM = 6;
uint8_t XP = 7;

#define MINPRESSURE 20
#define MAXPRESSURE 1000

void restaurarPinesTFT() {
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(XP, OUTPUT);
  pinMode(YM, OUTPUT);
}

TSPoint leerTouchRaw() {
  TSPoint p = {0, 0, 0};

  // 1. Leer X
  pinMode(YP, INPUT);
  pinMode(YM, INPUT);
  digitalWrite(YP, LOW);
  digitalWrite(YM, LOW);
  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  digitalWrite(XP, HIGH);
  digitalWrite(XM, LOW);
  int16_t s0 = analogRead(YP);
  int16_t s1 = analogRead(YP);
  if (abs(s0 - s1) > 15) { restaurarPinesTFT(); return p; }
  p.x = 1023 - ((s0 + s1) >> 1);

  // 2. Leer Y
  pinMode(XP, INPUT);
  pinMode(XM, INPUT);
  digitalWrite(XP, LOW);
  digitalWrite(XM, LOW);
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);
  digitalWrite(YP, HIGH);
  digitalWrite(YM, LOW);
  s0 = analogRead(XM);
  s1 = analogRead(XM);
  if (abs(s0 - s1) > 15) { restaurarPinesTFT(); return p; }
  p.y = 1023 - ((s0 + s1) >> 1);

  // 3. Leer Z (Presión)
  pinMode(XP, OUTPUT);
  digitalWrite(XP, LOW);
  pinMode(YM, OUTPUT);
  digitalWrite(YM, HIGH);
  digitalWrite(XM, LOW);
  pinMode(XM, INPUT);
  digitalWrite(YP, LOW);
  pinMode(YP, INPUT);
  int16_t z1 = analogRead(XM);
  int16_t z2 = analogRead(YP);
  if (z1 != 0) {
    p.z = (1023 - (z2 - z1));
  }

  restaurarPinesTFT();
  return p;
}

void autodetectarPinesTouch() {
  uint8_t ap[2], dp[2], found = 0;
  int16_t val[2];
  for (uint8_t i = A0; i < A4; i++) pinMode(i, INPUT_PULLUP);
  for (uint8_t i = 2; i < 10; i++)  pinMode(i, INPUT_PULLUP);

  for (uint8_t i = A0; i < A4; i++) {
    for (uint8_t j = 5; j < 10; j++) {
      pinMode(j, OUTPUT);
      digitalWrite(j, LOW);
      analogRead(i);
      int16_t v = analogRead(i);
      if (v < 100 && found < 2) {
        ap[found]  = i;
        dp[found]  = j;
        val[found] = v;
        found++;
      }
      pinMode(j, INPUT_PULLUP);
    }
  }
  for (uint8_t i = A0; i < A4; i++) pinMode(i, OUTPUT);
  for (uint8_t i = 2; i < 10; i++)  pinMode(i, OUTPUT);

  if (found == 2) {
    uint8_t ix = (val[0] < val[1]) ? 0 : 1;
    XM = ap[ix];  XP = dp[ix];
    YP = ap[!ix]; YM = dp[!ix];
  }
}

// ======================================================================================
// CALIBRACIÓN TÁCTIL FIJA PARA TU PANTALLA 3.5" (SIN DESCALIBRACIÓN ACCIDENTAL)
// ======================================================================================
struct TouchCalib {
  uint16_t magic;
  int16_t  xLeft, xRight, yTop, yBot;
  uint8_t  swapXY;
};

// Valores exactos de tu pantalla 3.5" en modo Horizontal (480x320)
TouchCalib tCal = { 0xEC63, 920, 150, 70, 830, 1 };

MCUFRIEND_kbv tft;
uint16_t g_lcdID = 0;

// ======================================================================================
// DEFINICIÓN DE PINES DE SENSORES Y SALIDAS (UNO & MEGA)
// ======================================================================================
// En MEGA puedes usar A8 para Voltaje sin tocar A4; en UNO usa A4
#if defined(__AVR_ATmega2560__)
  #define PIN_VOLTAJE A8
#else
  #define PIN_VOLTAJE A4
#endif
#define PIN_ACS712    A5
#define PIN_CKP       10
#define PIN_CMP       11
#define PIN_ECU_PWR   12

// ======================================================================================
// VARIABLES DE MEDICIÓN Y GENERADOR (OPTIMIZADAS EN ENTEROS: CENTIVOLTIOS / CENTIAMPERIOS)
// ======================================================================================
// Usamos enteros (1380 = 13.80V, 145 = 1.45A) para ahorrar >2KB de Flash en el UNO
// Escala del divisor de voltaje en centivoltios calibrada directamente con lectura real (26.60V reales)
#define ESCALA_DIVISOR_CV 2667UL

uint32_t adcZeroACS      = 65536UL; // ~2.50V acumulado en 128 muestras (512 * 128)
uint16_t voltCenti       = 0;     // Voltaje medido en centiVoltios
uint16_t voltNominalCenti= 1380;  // 13.80 V (respaldo si se desactiva ADC)
uint16_t ampCenti        = 0;     // 0.00 A
uint16_t ampPeakCenti    = 0;     // 0.00 A
uint16_t ampTripCenti    = 0;     // Corriente congelada en el instante del disparo por sobrecorriente
uint16_t wattDeci        = 0;     // 0.0 W (décimas de Watt)
uint16_t ampLimitCenti   = 50;    // 0.50 A (500mA) inicial por seguridad al conectar una ECU
bool     usarAdcVoltaje  = true;  // Lectura analógica del voltímetro en A4 habilitada
bool     overCurrentTrip = false;

enum PatronRueda : uint8_t {
  PATRON_60_2 = 0,
  PATRON_36_1 = 1,
  PATRON_36_2 = 2,
  PATRON_HALL = 3
};

volatile bool     genActivo        = false;
volatile uint8_t  patronActual     = PATRON_60_2;
volatile uint16_t rpmObjetivo      = 850;
volatile uint8_t  totalDientes     = 60;
volatile uint8_t  dientesFaltantes = 2;
volatile uint8_t  dienteActual     = 0;
volatile bool     medioDiente      = false;
volatile uint8_t  vueltaCiclo      = 0;
volatile bool     ecuPowerOn       = false; // Arranca con la salida apagada por seguridad y para calibrar el ACS712 en 0.00A real

// ======================================================================================
// PALETA DE COLORES (RGB565) Y CACHÉ DE PANTALLA
// ======================================================================================
#define C_BG          0x0842
#define C_PANEL       0x10A4
#define C_BORDER      0x39E7
#define C_TEXT        0xFFFF
#define C_MUTED       0x94B2
#define C_VOLT        0x07FF
#define C_AMP         0xFD20
#define C_PWR         0xFFE0
#define C_GREEN       0x07E0
#define C_DARKGREEN   0x0320
#define C_RED         0xF800
#define C_DARKRED     0x6000
#define C_BLUE_BTN    0x1A73
#define C_CYAN_DARK   0x0410

int16_t W = 480;
int16_t H = 320;

uint16_t lastDispV      = 0xFFFF;
uint16_t lastDispI      = 0xFFFF;
uint16_t lastDispP      = 0xFFFF;
uint16_t lastDispRPM    = 0;
uint8_t  lastDispPatron = 255;
bool     lastDispGen    = false;
bool     lastDispEcuPwr = false;
bool     lastDispTrip   = false;

unsigned long lastSensorMillis = 0;
unsigned long lastSerialMillis = 0;
unsigned long lastTouchMillis  = 0;

char serialBuf[36];
uint8_t serialIdx = 0;

void dibujarInterfazCompleta();

// ======================================================================================
// INTERRUPCIÓN DE HARDWARE TIMER1 (GENERACIÓN EXACTA CKP Y CMP EN UNO Y MEGA)
// ======================================================================================
#if defined(__AVR_ATmega2560__)
  // En Arduino MEGA: D10 es PB4 y D11 es PB5
  #define CKP_HIGH() (PORTB |=  (1 << PB4))
  #define CKP_LOW()  (PORTB &= ~(1 << PB4))
  #define CMP_HIGH() (PORTB |=  (1 << PB5))
  #define CMP_LOW()  (PORTB &= ~(1 << PB5))
#else
  // En Arduino UNO: D10 es PB2 y D11 es PB3
  #define CKP_HIGH() (PORTB |=  (1 << PB2))
  #define CKP_LOW()  (PORTB &= ~(1 << PB2))
  #define CMP_HIGH() (PORTB |=  (1 << PB3))
  #define CMP_LOW()  (PORTB &= ~(1 << PB3))
#endif

ISR(TIMER1_COMPA_vect) {
  if (!genActivo) {
    CKP_LOW();
    CMP_LOW();
    return;
  }

  uint8_t dientesActivos = totalDientes - dientesFaltantes;

  if (!medioDiente) {
    if (dienteActual < dientesActivos) CKP_HIGH();
    else                               CKP_LOW();

    uint8_t anchoPulsoCMP = (totalDientes >= 36) ? 6 : 1;
    if (vueltaCiclo == 0 && dienteActual >= 1 && dienteActual <= anchoPulsoCMP) CMP_HIGH();
    else                                                                        CMP_LOW();

    medioDiente = true;
  } else {
    CKP_LOW();
    medioDiente = false;
    dienteActual++;
    if (dienteActual >= totalDientes) {
      dienteActual = 0;
      vueltaCiclo ^= 1;
    }
  }
}

void actualizarTimerCKP() {
  switch (patronActual) {
    case PATRON_60_2: totalDientes = 60; dientesFaltantes = 2; break;
    case PATRON_36_1: totalDientes = 36; dientesFaltantes = 1; break;
    case PATRON_36_2: totalDientes = 36; dientesFaltantes = 2; break;
    case PATRON_HALL: totalDientes = 4;  dientesFaltantes = 0; break;
  }

  uint32_t divisor = (uint32_t)rpmObjetivo * (uint32_t)totalDientes;
  if (divisor == 0) divisor = 1;
  uint32_t ocr = (60000000UL / divisor) - 1;

  noInterrupts();
  if (ocr > 65535UL) {
    ocr = (7500000UL / divisor) - 1;
    if (ocr > 65535UL) ocr = 65535UL;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
  } else {
    if (ocr < 20UL) ocr = 20UL;
    TCCR1B = (1 << WGM12) | (1 << CS11);
  }
  OCR1A = (uint16_t)ocr;
  if (TCNT1 >= OCR1A) TCNT1 = 0;
  interrupts();
}

void inicializarTimer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 2000;
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  actualizarTimerCKP();
}

// ======================================================================================
// ASISTENTE DE CALIBRACIÓN TÁCTIL DE 3 PUNTOS (CORRIGE EJES CRUZADOS O INVERTIDOS)
// ======================================================================================
void esperarToquePromedio(int16_t &rx, int16_t &ry) {
  while (leerTouchRaw().z >= MINPRESSURE) delay(15);
  delay(120);

  int32_t sumX = 0, sumY = 0;
  uint8_t cnt = 0;
  while (cnt < 8) {
    TSPoint p = leerTouchRaw();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      sumX += p.x;
      sumY += p.y;
      cnt++;
      delay(12);
    } else {
      sumX = 0; sumY = 0; cnt = 0;
    }
  }
  rx = (int16_t)(sumX >> 3);
  ry = (int16_t)(sumY >> 3);
}

void dibujarCruz(int16_t x, int16_t y, uint16_t col) {
  tft.drawFastHLine(x - 10, y, 21, col);
  tft.drawFastVLine(x, y - 10, 21, col);
}

void ejecutarCalibracionTouch() {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_VOLT);
  tft.setTextSize(2);
  tft.setCursor(80, 120);
  tft.print(F("TOCA LAS 3 CRUCES VERDES"));

  int16_t x1, y1, x2, y2, x3, y3;
  dibujarCruz(40, 40, C_GREEN);
  esperarToquePromedio(x1, y1);
  dibujarCruz(40, 40, C_MUTED);

  dibujarCruz(W - 40, H - 40, C_GREEN);
  esperarToquePromedio(x2, y2);
  dibujarCruz(W - 40, H - 40, C_MUTED);

  dibujarCruz(W - 40, 40, C_PWR);
  esperarToquePromedio(x3, y3);

  // Detectar si los ejes X/Y están cruzados comparando el desplazamiento horizontal (Punto 1 -> Punto 3)
  int16_t ax1, ax2, ay1, ay2;
  if (abs(y3 - y1) > abs(x3 - x1)) {
    tCal.swapXY = 1;
    ax1 = y1; ax2 = y2; ay1 = x1; ay2 = x2;
  } else {
    tCal.swapXY = 0;
    ax1 = x1; ax2 = x2; ay1 = y1; ay2 = y2;
  }

  // Extrapolar 40px hacia los bordes (40/(480-80) = 1/10 en X, 40/(320-80) = 1/6 en Y)
  int16_t dx = (ax2 - ax1) / 10;
  int16_t dy = (ay2 - ay1) / 6;
  tCal.xLeft  = ax1 - dx;
  tCal.xRight = ax2 + dx;
  tCal.yTop   = ay1 - dy;
  tCal.yBot   = ay2 + dy;
  tCal.magic  = 0xEC60;

  EEPROM.put(0, tCal);
  dibujarInterfazCompleta();
}

// ======================================================================================
// LECTURA DE SENSORES EN ENTEROS (RÁPIDA Y COMPACTA)
// ======================================================================================
void leerSensores() {
  // 1. Pre-cargar y estabilizar el capacitor Sample & Hold del MUX en A5 para que el voltaje de A4 no afecte el span
  for (uint8_t k = 0; k < 4; k++) analogRead(PIN_ACS712);
  delayMicroseconds(80);

  uint32_t sumaI = 0;
  // Sobremuestreo de 128 lecturas para máxima estabilidad con el ACS712 de 30A (66 mV/A)
  for (uint8_t i = 0; i < 128; i++) sumaI += analogRead(PIN_ACS712);

  // Calibración de 2 puntos (Cero y Span) basada en mediciones reales (0.266A y 0.559A):
  // Corrige tanto la ganancia de pendiente como la desviación a corrientes altas
  int32_t diff128 = abs((int32_t)sumaI - (int32_t)adcZeroACS);
  uint16_t iBruta = (uint16_t)(((uint32_t)diff128 * 100UL) / 1734UL);
  uint16_t iMed   = 0;
  if (iBruta >= 4) {
    // Ajuste lineal de Span: a iBruta=24 -> 27 (0.27A), a iBruta=68 -> 56 (0.56A)
    if (iBruta <= 24) {
      iMed = (uint16_t)(((uint32_t)iBruta * 266UL) / 240UL);
    } else {
      iMed = 27 + (uint16_t)(((uint32_t)(iBruta - 24) * 293UL) / 440UL);
    }
  }

  // 1. PROTECCIÓN INSTANTÁNEA DE FUSIBLE ELECTRÓNICO (evalúa la lectura inmediata iMed sin retardo de filtro)
  if (iMed >= ampLimitCenti && ecuPowerOn) {
    digitalWrite(PIN_ECU_PWR, LOW); // Corta el relé en el acto (< 15 ms)
    ecuPowerOn      = false;
    overCurrentTrip = true;
    ampCenti        = iMed;
    ampTripCenti    = iMed;         // Congelar el valor exacto al que se disparó la protección
    if (iMed > ampPeakCenti) ampPeakCenti = iMed;
  } else {
    // 2. Filtro Adaptativo Inteligente:
    // - Si hay un cambio brusco de corriente (>= 0.08A, ej. al conectar/desconectar carga), salta de inmediato a iMed.
    // - Si la corriente está fija (variación < 0.08A), aplica filtro suave (3/4 + 1/4) con redondeo para no oscilar.
    if (abs((int16_t)iMed - (int16_t)ampCenti) >= 8) {
      ampCenti = iMed;
    } else {
      ampCenti = (uint16_t)(((uint32_t)ampCenti * 3UL + iMed + 2UL) >> 2);
    }
    if (ampCenti > ampPeakCenti) ampPeakCenti = ampCenti;
  }

  if (usarAdcVoltaje) {
    pinMode(PIN_VOLTAJE, INPUT);
    analogRead(PIN_VOLTAJE);
    uint16_t sumaV = 0;
    for (uint8_t i = 0; i < 16; i++) sumaV += analogRead(PIN_VOLTAJE);
    // Con suma de 16 muestras (máx = 1023 * 16 = 16368) -> precisión de centésimas sin pérdida por truncamiento
    uint16_t vMed = (uint16_t)(((uint32_t)sumaV * ESCALA_DIVISOR_CV) / 16368UL);
    if (vMed < 15) vMed = 0; // Zona muerta inferior < 0.15V
    // Filtro adaptativo también en Voltaje para respuesta rápida al mover el potenciómetro
    if (abs((int16_t)vMed - (int16_t)voltCenti) >= 15) {
      voltCenti = vMed;
    } else {
      voltCenti = (uint16_t)(((uint32_t)voltCenti * 3UL + vMed + 2UL) >> 2);
    }
  } else {
    #if !defined(__AVR_ATmega2560__)
      pinMode(A4, OUTPUT);
      digitalWrite(A4, HIGH);
    #endif
    voltCenti = ecuPowerOn ? voltNominalCenti : 0;
  }

  // Potencia en décimas de Watt: (voltCenti * ampCenti) / 1000
  wattDeci = (uint16_t)(((uint32_t)voltCenti * (uint32_t)ampCenti) / 1000UL);

  if (ampCenti >= ampLimitCenti && ecuPowerOn) {
    digitalWrite(PIN_ECU_PWR, LOW);
    ecuPowerOn      = false;
    overCurrentTrip = true;
    ampTripCenti    = ampCenti;
  }
}

void calibrarCeroACS712() {
  uint32_t suma = 0;
  analogRead(PIN_ACS712);
  // Promedio de 2 bloques de 128 muestras (256 lecturas) para fijar el cero exacto en escala x128
  for (uint16_t i = 0; i < 256; i++) suma += analogRead(PIN_ACS712);
  adcZeroACS   = suma >> 1; // Equivale a la suma de 128 muestras en reposo (~65536)
  ampCenti     = 0;
  ampPeakCenti = 0;
  // Guardar la tara en EEPROM (dirección 20) para conservarla al apagar el equipo
  uint16_t magicTara = 0xAC71;
  EEPROM.put(20, magicTara);
  EEPROM.put(22, adcZeroACS);
}

// ======================================================================================
// IMPRESIÓN FORMATEADA DE ENTEROS (SIN USAR FLOAT NI DTOSTRF)
// ======================================================================================
void printCenti(Print &out, uint16_t val) {
  uint16_t ent = val / 100;
  uint8_t  dec = val % 100;
  if (ent < 10) out.print(' ');
  out.print(ent);
  out.print('.');
  if (dec < 10) out.print('0');
  out.print(dec);
}

// ======================================================================================
// TELEMETRÍA Y CONTROL POR PUERTO SERIAL (115200 BAUDIOS)
// ======================================================================================
const char* obtenerNombreCortoPatron(uint8_t pat) {
  switch (pat) {
    case PATRON_60_2: return "60-2";
    case PATRON_36_1: return "36-1";
    case PATRON_36_2: return "36-2";
    default:          return "HALL4";
  }
}

void enviarTelemetriaSerial() {
  uint32_t freqHz = ((uint32_t)rpmObjetivo * (uint32_t)totalDientes) / 60UL;
  Serial.print(F("DATA,V="));    printCenti(Serial, voltCenti);
  Serial.print(F(",I="));        printCenti(Serial, ampCenti);
  Serial.print(F(",P="));        Serial.print(wattDeci / 10); Serial.print('.'); Serial.print(wattDeci % 10);
  Serial.print(F(",PK="));       printCenti(Serial, ampPeakCenti);
  Serial.print(F(",IMX="));      printCenti(Serial, ampLimitCenti);
  Serial.print(F(",RPM="));      Serial.print(rpmObjetivo);
  Serial.print(F(",FREQ="));     Serial.print(freqHz);
  Serial.print(F(",PAT="));      Serial.print(obtenerNombreCortoPatron(patronActual));
  Serial.print(F(",GEN="));      Serial.print(genActivo ? 1 : 0);
  Serial.print(F(",PWR="));      Serial.print(ecuPowerOn ? 1 : 0);
  Serial.print(F(",TRIP="));     Serial.println(overCurrentTrip ? 1 : 0);
}

void ejecutarComandoSerial(char* cmd) {
  if (strncmp(cmd, "RPM=", 4) == 0) {
    rpmObjetivo = (uint16_t)constrain(atol(cmd + 4), 150, 8000);
    actualizarTimerCKP();
  } else if (strncmp(cmd, "PAT=", 4) == 0) {
    uint8_t p = (uint8_t)atoi(cmd + 4);
    if (p <= 3) { patronActual = p; dienteActual = 0; vueltaCiclo = 0; actualizarTimerCKP(); }
  } else if (strncmp(cmd, "GEN=", 4) == 0) {
    genActivo = (atoi(cmd + 4) != 0);
    if (!genActivo) { digitalWrite(PIN_CKP, LOW); digitalWrite(PIN_CMP, LOW); }
    else { dienteActual = 0; vueltaCiclo = 0; medioDiente = false; actualizarTimerCKP(); }
  } else if (strncmp(cmd, "PWR=", 4) == 0) {
    ecuPowerOn = (atoi(cmd + 4) != 0);
    if (ecuPowerOn) overCurrentTrip = false;
    digitalWrite(PIN_ECU_PWR, ecuPowerOn ? HIGH : LOW);
  } else if (strncmp(cmd, "ADCV=", 5) == 0) {
    usarAdcVoltaje = (atoi(cmd + 5) != 0);
  } else if (strncmp(cmd, "ILIM=", 5) == 0) {
    // Recibe límite en centiamperios (ej. ILIM=300 -> 3.00A) o en Amperios si es <= 5 (ej. ILIM=3 -> 3.00A)
    uint16_t val = (uint16_t)atoi(cmd + 5);
    if (val <= 5) val *= 100;
    ampLimitCenti = constrain(val, 50, 500);
  } else if (strcmp(cmd, "TARE") == 0) {
    calibrarCeroACS712();
  } else if (strcmp(cmd, "PEAKRST") == 0) {
    ampPeakCenti = 0;
  } else if (strcmp(cmd, "CAL") == 0) {
    ejecutarCalibracionTouch();
  }
}

void procesarPuertoSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        ejecutarComandoSerial(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

// ======================================================================================
// INTERFAZ GRÁFICA DE USUARIO (UI) EN PANTALLA 3.5" (480x320)
// ======================================================================================
void dibujarBoton(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t cf, uint16_t cb, const char* txt, uint8_t tam, uint16_t ct) {
  tft.fillRoundRect(x, y, w, h, 6, cf);
  tft.drawRoundRect(x, y, w, h, 6, cb);
  int16_t tx = x + (w - ((int16_t)strlen(txt) * 6 * tam)) / 2;
  int16_t ty = y + (h - (8 * tam)) / 2;
  tft.setCursor(tx, ty);
  tft.setTextColor(ct);
  tft.setTextSize(tam);
  tft.print(txt);
}

void actualizarMedidoresUI(bool forzar) {
  if (forzar || abs((int16_t)voltCenti - (int16_t)lastDispV) >= 5) {
    lastDispV = voltCenti;
    tft.setTextColor(C_VOLT, C_PANEL);
    tft.setTextSize(3);
    tft.setCursor(22, 65);
    printCenti(tft, voltCenti);
    tft.setTextSize(2);
    tft.print(F(" V"));

    int16_t barW = (int16_t)(((uint32_t)constrain(voltCenti, 0, 2800) * 196UL) / 2800UL);
    uint16_t barCol = (voltCenti >= 1150 && voltCenti <= 1480) ? C_GREEN : (voltCenti >= 2300 && voltCenti <= 2850 ? C_VOLT : C_PWR);
    tft.fillRect(22, 95, barW, 8, barCol);
    tft.fillRect(22 + barW, 95, 196 - barW, 8, C_BG);
  }

  uint16_t dispAmp = overCurrentTrip ? ampTripCenti : ampCenti;
  if (forzar || overCurrentTrip != lastDispTrip || abs((int16_t)dispAmp - (int16_t)lastDispI) >= 3) {
    lastDispI = dispAmp;
    uint16_t colI = (overCurrentTrip || dispAmp > ((ampLimitCenti * 4) / 5)) ? C_RED : C_AMP;
    tft.setTextColor(colI, C_PANEL);
    tft.setTextSize(3);
    tft.setCursor(22, 132);
    printCenti(tft, dispAmp);
    tft.setTextSize(2);
    tft.print(F(" A"));

    // Botón compacto para cambiar el límite de protección (Fusible)
    tft.drawRoundRect(142, 128, 74, 24, 4, C_VOLT);
    tft.setTextSize(1);
    tft.setTextColor(C_VOLT, C_PANEL);
    tft.setCursor(148, 136);
    tft.print(F("LIM:"));
    tft.print(ampLimitCenti / 100);
    tft.print('.');
    tft.print((ampLimitCenti % 100) / 10);
    tft.print(F("A "));

    int16_t barW = (int16_t)(((uint32_t)constrain(dispAmp, 0, ampLimitCenti) * 196UL) / ampLimitCenti);
    tft.fillRect(22, 162, barW, 8, colI);
    tft.fillRect(22 + barW, 162, 196 - barW, 8, C_BG);
  }

  if (forzar || wattDeci != lastDispP) {
    lastDispP = wattDeci;
    tft.setTextColor(C_PWR, C_PANEL);
    tft.setTextSize(2);
    tft.setCursor(22, 192);
    if (wattDeci < 100) tft.print(' ');
    tft.print(wattDeci / 10);
    tft.print('.');
    tft.print(wattDeci % 10);
    tft.print(F(" W "));

    tft.setTextColor(C_MUTED, C_PANEL);
    tft.setTextSize(1);
    tft.setCursor(135, 198);
    tft.print(F("PK:"));
    printCenti(tft, ampPeakCenti);
    tft.print('A');
  }

  if (forzar || ecuPowerOn != lastDispEcuPwr || overCurrentTrip != lastDispTrip) {
    lastDispEcuPwr = ecuPowerOn;
    lastDispTrip   = overCurrentTrip;
    if (overCurrentTrip) {
      dibujarBoton(20, 260, 200, 42, C_RED, C_TEXT, "RESET SALIDA", 2, C_TEXT);
    } else if (ecuPowerOn) {
      dibujarBoton(20, 260, 200, 42, C_DARKGREEN, C_GREEN, "SALIDA ACTIVA", 2, C_GREEN);
    } else {
      dibujarBoton(20, 260, 200, 42, C_DARKRED, C_RED, "SALIDA APAGADA", 2, C_MUTED);
    }
  }
}

void actualizarGeneradorUI(bool forzar) {
  bool cambioPatron = (patronActual != lastDispPatron);

  if (forzar || cambioPatron) {
    lastDispPatron = patronActual;
    const char* np = "< 60-2 + CMP (UNIV) >";
    if (patronActual == PATRON_36_1)      np = "< 36-1 + CMP (FORD) >";
    else if (patronActual == PATRON_36_2) np = "< 36-2 + CMP (TOYO) >";
    else if (patronActual == PATRON_HALL) np = "< 4 PULSOS (HALL)   >";
    dibujarBoton(252, 58, 206, 36, C_BLUE_BTN, C_VOLT, np, 1, C_TEXT);
  }

  if (forzar || rpmObjetivo != lastDispRPM || cambioPatron) {
    lastDispRPM = rpmObjetivo;
    tft.setTextColor(C_GREEN, C_PANEL);
    tft.setTextSize(4);
    tft.setCursor(268, 112);
    if (rpmObjetivo < 1000) tft.print(' ');
    tft.print(rpmObjetivo);

    tft.setTextSize(2);
    tft.setTextColor(C_MUTED, C_PANEL);
    tft.setCursor(375, 126);
    tft.print(F("RPM"));

    uint32_t freqHz = ((uint32_t)rpmObjetivo * (uint32_t)totalDientes) / 60UL;
    tft.setTextSize(1);
    tft.setTextColor(C_VOLT, C_PANEL);
    tft.setCursor(268, 150);
    tft.print(F("Frec CKP: "));
    tft.print(freqHz);
    tft.print(F(" Hz   "));

    tft.setTextColor(C_PWR, C_PANEL);
    tft.setCursor(370, 150);
    tft.print(F("CMP: 1/720"));
  }

  if (forzar || genActivo != lastDispGen) {
    lastDispGen = genActivo;
    if (genActivo) {
      dibujarBoton(252, 260, 206, 42, C_GREEN, C_TEXT, "CKP/CMP: GENERANDO", 2, C_BG);
      tft.fillCircle(458, 16, 7, C_GREEN);
    } else {
      dibujarBoton(252, 260, 206, 42, C_DARKRED, C_RED, "INICIAR CKP / CMP", 2, C_TEXT);
      tft.fillCircle(458, 16, 7, C_RED);
    }
  }
}

void dibujarInterfazCompleta() {
  tft.fillScreen(C_BG);

  tft.fillRect(0, 0, W, 32, C_PANEL);
  tft.drawFastHLine(0, 32, W, C_VOLT);
  tft.setTextColor(C_TEXT);
  tft.setTextSize(2);
  tft.setCursor(12, 8);
  tft.print(F("BANCO PROBADOR DE ECUs"));
  tft.setTextSize(1);
  tft.setTextColor(C_VOLT);
  tft.setCursor(320, 12);
  tft.print(F("FUENTE + CKP/CMP"));

  tft.fillRoundRect(10, 40, 220, 270, 8, C_PANEL);
  tft.drawRoundRect(10, 40, 220, 270, 8, C_BORDER);

  tft.setTextSize(1);
  tft.setTextColor(C_MUTED);
  tft.setCursor(22, 48);  tft.print(F("VOLTAJE SUMINISTRO"));
  tft.setCursor(22, 115); tft.print(F("CORRIENTE"));
  tft.setCursor(22, 178); tft.print(F("POTENCIA DE CONSUMO"));

  dibujarBoton(20, 218, 96, 32, C_BLUE_BTN, C_BORDER, "TARA 0.0A", 1, C_TEXT);
  dibujarBoton(124, 218, 96, 32, C_BLUE_BTN, C_BORDER, "BORRAR PK", 1, C_TEXT);

  tft.fillRoundRect(240, 40, 230, 270, 8, C_PANEL);
  tft.drawRoundRect(240, 40, 230, 270, 8, C_BORDER);

  tft.setTextSize(1);
  tft.setTextColor(C_MUTED);
  tft.setCursor(252, 46); tft.print(F("RUEDA FONICA (D10=CKP | D11=CMP)"));
  tft.drawRoundRect(252, 102, 206, 62, 5, C_BORDER);

  dibujarBoton(252, 172, 48, 36, C_BLUE_BTN, C_BORDER, "-500", 1, C_TEXT);
  dibujarBoton(305, 172, 48, 36, C_BLUE_BTN, C_BORDER, "-50",  1, C_TEXT);
  dibujarBoton(358, 172, 48, 36, C_BLUE_BTN, C_BORDER, "+50",  1, C_TEXT);
  dibujarBoton(410, 172, 48, 36, C_BLUE_BTN, C_BORDER, "+500", 1, C_TEXT);

  dibujarBoton(252, 216, 48, 34, C_CYAN_DARK, C_VOLT, "250",  1, C_VOLT);
  dibujarBoton(305, 216, 48, 34, C_CYAN_DARK, C_VOLT, "850",  1, C_VOLT);
  dibujarBoton(358, 216, 48, 34, C_CYAN_DARK, C_VOLT, "2500", 1, C_VOLT);
  dibujarBoton(410, 216, 48, 34, C_CYAN_DARK, C_VOLT, "5000", 1, C_VOLT);

  actualizarMedidoresUI(true);
  actualizarGeneradorUI(true);
}

// ======================================================================================
// LECTURA Y GESTIÓN DE EVENTOS TÁCTILES
// ======================================================================================
void procesarTouch() {
  TSPoint p = leerTouchRaw();
  if (p.z < MINPRESSURE || p.z > MAXPRESSURE) return;
  if (millis() - lastTouchMillis < 180) return;
  lastTouchMillis = millis();

  int16_t rawX = tCal.swapXY ? p.y : p.x;
  int16_t rawY = tCal.swapXY ? p.x : p.y;

  int16_t tx = map(rawX, tCal.xLeft, tCal.xRight, 0, W);
  int16_t ty = map(rawY, tCal.yTop,  tCal.yBot,   0, H);
  tx = constrain(tx, 0, W - 1);
  ty = constrain(ty, 0, H - 1);

  // Botón para cambiar el límite de corriente (LIM: X.XA en x: 130..225, y: 118..165)
  // Escalones para fuente con LM2596 (máx 5.0A): 0.5A -> 1.0A -> 2.0A -> 3.0A -> 3.5A -> 5.0A
  if (tx >= 130 && tx <= 225 && ty >= 118 && ty <= 165 && p.z > 80) {
    if (ampLimitCenti < 80)        ampLimitCenti = 100;  // 1.0 A
    else if (ampLimitCenti < 150)  ampLimitCenti = 200;  // 2.0 A
    else if (ampLimitCenti < 250)  ampLimitCenti = 300;  // 3.0 A (Nominal máx LM2596)
    else if (ampLimitCenti < 325)  ampLimitCenti = 350;  // 3.5 A
    else if (ampLimitCenti < 420)  ampLimitCenti = 500;  // 5.0 A (Tope máximo)
    else                           ampLimitCenti = 50;   // 0.5 A (500 mA)
    actualizarMedidoresUI(true);
    return;
  }

  if (tx >= 20 && tx <= 116 && ty >= 215 && ty <= 252) {
    calibrarCeroACS712();
    actualizarMedidoresUI(true);
    return;
  }

  if (tx >= 124 && tx <= 220 && ty >= 215 && ty <= 252) {
    ampPeakCenti = 0;
    actualizarMedidoresUI(true);
    return;
  }

  if (tx >= 20 && tx <= 220 && ty >= 258 && ty <= 305) {
    if (overCurrentTrip) { overCurrentTrip = false; ecuPowerOn = true; }
    else { ecuPowerOn = !ecuPowerOn; }
    digitalWrite(PIN_ECU_PWR, ecuPowerOn ? HIGH : LOW);
    actualizarMedidoresUI(true);
    return;
  }

  if (tx >= 250 && tx <= 460 && ty >= 54 && ty <= 98) {
    patronActual = (patronActual + 1) % 4;
    noInterrupts(); dienteActual = 0; vueltaCiclo = 0; interrupts();
    actualizarTimerCKP();
    actualizarGeneradorUI(true);
    return;
  }

  if (ty >= 170 && ty <= 210) {
    int32_t nr = rpmObjetivo;
    if (tx >= 250 && tx <= 302)      nr -= 500;
    else if (tx >= 303 && tx <= 355) nr -= 50;
    else if (tx >= 356 && tx <= 408) nr += 50;
    else if (tx >= 409 && tx <= 462) nr += 500;
    rpmObjetivo = (uint16_t)constrain(nr, 150, 8000);
    actualizarTimerCKP();
    actualizarGeneradorUI(false);
    return;
  }

  if (ty >= 214 && ty <= 252) {
    if (tx >= 250 && tx <= 302)      rpmObjetivo = 250;
    else if (tx >= 303 && tx <= 355) rpmObjetivo = 850;
    else if (tx >= 356 && tx <= 408) rpmObjetivo = 2500;
    else if (tx >= 409 && tx <= 462) rpmObjetivo = 5000;
    actualizarTimerCKP();
    actualizarGeneradorUI(false);
    return;
  }

  if (tx >= 250 && tx <= 460 && ty >= 258 && ty <= 305) {
    genActivo = !genActivo;
    if (!genActivo) {
      digitalWrite(PIN_CKP, LOW);
      digitalWrite(PIN_CMP, LOW);
    } else {
      noInterrupts(); dienteActual = 0; vueltaCiclo = 0; medioDiente = false; interrupts();
      actualizarTimerCKP();
    }
    actualizarGeneradorUI(true);
    return;
  }
}

// ======================================================================================
// SETUP PRINCIPAL Y LOOP
// ======================================================================================
void setup() {
  // Esperar 300ms a que la fuente de alimentación estabilice los 5V antes de inicializar periféricos
  delay(300);
  Serial.begin(115200);

  pinMode(PIN_CKP, OUTPUT);
  pinMode(PIN_CMP, OUTPUT);
  pinMode(PIN_ECU_PWR, OUTPUT);
  digitalWrite(PIN_CKP, LOW);
  digitalWrite(PIN_CMP, LOW);
  digitalWrite(PIN_ECU_PWR, LOW); // Inicia con el relé abierto (SALIDA APAGADA) para calibrar 0.00A sin carga
  pinMode(PIN_ACS712, INPUT);
  pinMode(PIN_VOLTAJE, INPUT);

  autodetectarPinesTouch();

  tft.reset();
  g_lcdID = tft.readID();
  if (g_lcdID == 0xD3D3 || g_lcdID == 0xFFFF || g_lcdID == 0x0000) g_lcdID = 0x9486;

  tft.begin(g_lcdID);
  tft.setRotation(1);
  W = tft.width();
  H = tft.height();

  // Usamos siempre la calibración fija comprobada de tu pantalla (150, 920, 150, 940, swap=1)
  // ignorando cualquier calibración errónea previa en EEPROM.

  // Cargar la tara del ACS712 guardada en EEPROM (si ya se calibró con TARA 0.0A); si no, calibrar al inicio
  uint16_t magicTara = 0;
  EEPROM.get(20, magicTara);
  if (magicTara == 0xAC71) {
    EEPROM.get(22, adcZeroACS);
  } else {
    calibrarCeroACS712();
  }
  dibujarInterfazCompleta();

  inicializarTimer1();
}

void loop() {
  procesarPuertoSerial();
  procesarTouch();

  unsigned long ahora = millis();
  if (ahora - lastSensorMillis >= 35) {
    lastSensorMillis = ahora;
    leerSensores();
    actualizarMedidoresUI(false);
    actualizarGeneradorUI(false);
  }

  if (ahora - lastSerialMillis >= 200) {
    lastSerialMillis = ahora;
    enviarTelemetriaSerial();
  }
}
