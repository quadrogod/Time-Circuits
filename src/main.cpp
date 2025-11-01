/*****************************************************************
  Time-Circuits 15×7-Segment (Arduino Mega 2560 + 8×74HC595)

  Destination (sevseg1..5, RED)
  Present     (sevseg6..10, GREEN)
  Last        (sevseg11..15, YELLOW: порядок MM DD YYYY HH MM)
*****************************************************************/

#include <Arduino.h>
#include <Keypad.h>

/* --- Пины 74HC595 ----------------------------------------- */
#define DATA_PIN      22
#define LATCH_PIN     23
#define CLOCK_PIN     24

/* --- LED индикаторы --------------------------------------- */
#define LED_AM_DEST   30
#define LED_PM_DEST   31
#define LED_SEC1_DEST 32
#define LED_SEC2_DEST 33

#define LED_AM_PRES   34
#define LED_PM_PRES   35
#define LED_SEC1_PRES 36
#define LED_SEC2_PRES 37

#define LED_AM_LAST   38
#define LED_PM_LAST   39
#define LED_SEC1_LAST 40
#define LED_SEC2_LAST 41

/* --- Датчик ----------------------------------------------- */
#define NTC_PIN       A0

/* --- Клавиатура 4×4 --------------------------------------- */
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','D'},
  {'4','5','6','P'},
  {'7','8','9','L'},
  {'R','0','#','E'}
};
byte rowPins[ROWS] = {2,3,4,5};
byte colPins[COLS] = {6,7,8,9};
Keypad kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/* --- Мультиплекс ------------------------------------------ */
#define SR_CNT        8        // sr1..sr8
#define DIGITS_TOTAL  42       // общее число разрядов
#define DIGIT_US      1500UL   // микросекунды на один разряд
#define BLINK_MS      1000UL
#define DEBOUNCE_MS   200UL

/* Паттерны сегментов (общий катод: 1=сегмент включен) */
enum {D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D_MINUS,D_BLANK};
// static const byte SEG_PAT[12] = {
//   0b00111111, // 0
//   0b00000110, // 1
//   0b01011011, // 2
//   0b01001111, // 3
//   0b01100110, // 4
//   0b01101101, // 5
//   0b01111101, // 6
//   0b00000111, // 7
//   0b01111111, // 8
//   0b01101111, // 9
//   0b01000000, // -
//   0b00000000  // blank
// };

// Паттерны сегментов (общий АНОД: 0=сегмент включен, инвертировано)
static const byte SEG_PAT[12] = {
  0b11000000, // 0
  0b11111001, // 1
  0b10100100, // 2
  0b10110000, // 3
  0b10011001, // 4
  0b10010010, // 5
  0b10000010, // 6
  0b11111000, // 7
  0b10000000, // 8
  0b10010000, // 9
  0b10111111, // -
  0b11111111  // blank
};

/* --- Время ------------------------------------------------- */
struct DateTime {
  int m, d, y, h, min;
  bool valid;
  DateTime() { m=d=y=h=min=0; valid=false; }

  String toText() const {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02d.%02d.%04d %02d:%02d", d, m, y, h, min);
    return String(buf);
  }
};

DateTime destT, presT, lastT;

/* --- Глобальные ------------------------------------------- */
byte buf[DIGITS_TOTAL];      // 42 разряда, 0..41
byte curDig = 0;
unsigned long tDig=0, tBlink=0, tKey=0, tMin=0;
bool blinkTick = false;
bool jumpLock = false;

enum Mode {NONE, SET_DEST, SET_PRES, SET_LAST};
Mode mode = NONE;
String inDigits;

char lastKeyPressed = '\0';

const float BETA_THERM = 3950;

/* --- Утилиты ---------------------------------------------- */
bool isLeap(int y) {
  return (y%4==0 && y%100!=0) || (y%400==0);
}

bool dateOk(int M, int D, int Y, int h, int m) {
  if(M<1 || M>12 || D<1 || h>23 || m>59 || Y<0 || Y>9999) return false;
  int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if(M==2 && isLeap(Y)) dim[1] = 29;
  return D <= dim[M-1];
}

DateTime parse12(const String& s) {
  DateTime d;
  if(s.length() != 12) return d;
  d.m   = s.substring(0,2).toInt();
  d.d   = s.substring(2,4).toInt();
  d.y   = s.substring(4,8).toInt();
  d.h   = s.substring(8,10).toInt();
  d.min = s.substring(10,12).toInt();
  d.valid = dateOk(d.m, d.d, d.y, d.h, d.min);
  return d;
}

void to12h(int h24, int& h12out, bool& pm) {
  pm = (h24 >= 12);
  h12out = h24 % 12;
  if(!h12out) h12out = 12;
}

/* Таблица соответствия каждому разряду (0..41) конкретного регистра и бита. */
struct Sel { byte srIdx; byte bitMask; };
// static const Sel SEL[DIGITS_TOTAL] PROGMEM = {
//   {1,1<<0},{1,1<<1},{1,1<<2},{1,1<<3}, {1,1<<4},{1,1<<5},
//   {1,1<<6},{1,1<<7},{2,1<<0},{2,1<<1}, {2,1<<2},{2,1<<3},
//   {2,1<<4},{2,1<<5},{2,1<<6},{2,1<<7}, {3,1<<0},{3,1<<1},
//   {3,1<<2},{3,1<<3},{3,1<<4},{3,1<<5}, {3,1<<6},{3,1<<7},
//   {4,1<<0},{4,1<<1},{4,1<<2},{4,1<<3}, {4,1<<4},{4,1<<5},
//   {4,1<<6},{4,1<<7},{5,1<<0},{5,1<<1}, {5,1<<2},{5,1<<3},
//   {5,1<<4},{5,1<<5},{5,1<<6},{5,1<<7}, {6,1<<0},{6,1<<1},
// };
static const Sel SEL[DIGITS_TOTAL] PROGMEM = {
  {0,1<<0},{0,1<<1},{0,1<<2},{0,1<<3}, {0,1<<4},{0,1<<5},
  {0,1<<6},{0,1<<7},{1,1<<0},{1,1<<1}, {1,1<<2},{1,1<<3},
  {1,1<<4},{1,1<<5},{1,1<<6},{1,1<<7}, {2,1<<0},{2,1<<1},
  {2,1<<2},{2,1<<3},{2,1<<4},{2,1<<5}, {2,1<<6},{2,1<<7},
  {3,1<<0},{3,1<<1},{3,1<<2},{3,1<<3}, {3,1<<4},{3,1<<5},
  {3,1<<6},{3,1<<7},{4,1<<0},{4,1<<1}, {4,1<<2},{4,1<<3},
  {4,1<<4},{4,1<<5},{4,1<<6},{4,1<<7}, {5,1<<0},{5,1<<1},
};

/* Вывод 8 реесторов в 74HC595 */
void latchAll(const byte* data) {
  digitalWrite(LATCH_PIN, LOW);
  for (int8_t i = SR_CNT-1; i >= 0; --i) {
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[i]);
  }
  digitalWrite(LATCH_PIN, HIGH);
}

/* Показываем разряд idx - один из 42 */
void showDigit(byte idx) {
  static const byte CLR[SR_CNT] = {0};
  byte sr[SR_CNT] = {0};

  // Сегменты - sr1 (0)
  byte sym = buf[idx];
  if (sym > D_BLANK) sym = D_BLANK;
  sr[0] = SEG_PAT[sym];

  // COM - один активный бит в одном из sr2..sr7, по таблице
  Sel sel;
  memcpy_P(&sel, &SEL[idx], sizeof(Sel));
  sr[ 1 + sel.srIdx ] = sel.bitMask;

  latchAll(sr);
  delayMicroseconds(DIGIT_US);
  latchAll(CLR);
}

void put2(byte off, int value) {
  buf[off] = (value / 10) % 10;
  buf[off + 1] = value % 10;
}

void put4(byte off, int value) {
  buf[off] = (value / 1000) % 10;
  buf[off + 1] = (value / 100) % 10;
  buf[off + 2] = (value / 10) % 10;
  buf[off + 3] = value % 10;
}

void fillDash(byte from, byte count) {
  for (byte i = 0; i < count; i++) buf[from + i] = D_MINUS;
}

void fillBlank(byte from, byte count) {
  for (byte i = 0; i < count; i++) buf[from + i] = D_BLANK;
}

/* Буфер для Destination Time */
void toBufferDest(const DateTime& dt) {
  if (!dt.valid) {
    fillDash(0, 14);
    return;
  }
  fillBlank(0, 2); put2(2, dt.m);
  put2(4, dt.d);
  put4(6, dt.y);
  int h12; bool pm; to12h(dt.h, h12, pm);
  put2(10, h12);
  put2(12, dt.min);
}

/* Буфер для Present Time */
void toBufferPres(const DateTime& dt) {
  if (!dt.valid) {
    fillDash(14, 14);
    return;
  }
  fillBlank(14, 2); put2(16, dt.m);
  put2(18, dt.d);
  put4(20, dt.y);
  int h12; bool pm; to12h(dt.h, h12, pm);
  put2(24, h12);
  put2(26, dt.min);
}

/* Буфер для Last Time (та же логика, другой диапазон) */
void toBufferLast(const DateTime& dt) {
  if (!dt.valid) {
    fillDash(28, 14);
    return;
  }
  fillBlank(28, 2); put2(30, dt.m);
  put2(32, dt.d);
  put4(34, dt.y);
  int h12; bool pm; to12h(dt.h, h12, pm);
  put2(38, h12);
  put2(40, dt.min);
}

void refreshAll() {
  toBufferDest(destT);
  toBufferPres(presT);
  toBufferLast(lastT);
}

// Управление LED индикаторами
void setLeds(const DateTime& dt, int pinAM, int pinPM, int pinS1, int pinS2, bool blinkSecs) {
  if (!dt.valid) {
    digitalWrite(pinAM, LOW);
    digitalWrite(pinPM, LOW);
    digitalWrite(pinS1, LOW);
    digitalWrite(pinS2, LOW);
    return;
  }
  bool pm = (dt.h >= 12);
  digitalWrite(pinAM, pm ? LOW : HIGH);
  digitalWrite(pinPM, pm ? HIGH : LOW);
  if (blinkSecs) {
    digitalWrite(pinS1, blinkTick ? HIGH : LOW);
    digitalWrite(pinS2, blinkTick ? HIGH : LOW);
  } else {
    digitalWrite(pinS1, HIGH);
    digitalWrite(pinS2, HIGH);
  }
}

// Считывание температуры с NTC
float getT() {
  int raw = analogRead(NTC_PIN);
  // float voltage = raw * (5.0 / 1023.0);
  // float tempC = (voltage - 2.5f) * 20.0f + 24.0f;  //приблизительно для Wokwi
  float tempC = 1 / (log(1 / (1023. / raw - 1)) / BETA_THERM + 1.0 / 298.15) - 273.15;
  return tempC;
}

void setup() {
  Serial.begin(115200);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  // for (int p : {LED_AM_DEST, LED_PM_DEST, LED_SEC1_DEST, LED_SEC2_DEST,
  //               LED_AM_PRES, LED_PM_PRES, LED_SEC1_PRES, LED_SEC2_PRES,
  //               LED_AM_LAST, LED_PM_LAST, LED_SEC1_LAST, LED_SEC2_LAST}) pinMode(p, OUTPUT);

  const byte ledPins[] = {
    LED_AM_DEST, LED_PM_DEST, LED_SEC1_DEST, LED_SEC2_DEST,
    LED_AM_PRES, LED_PM_PRES, LED_SEC1_PRES, LED_SEC2_PRES,
    LED_AM_LAST, LED_PM_LAST, LED_SEC1_LAST, LED_SEC2_LAST
  };
  for (byte i = 0; i < sizeof(ledPins); i++) pinMode(ledPins[i], OUTPUT);

  refreshAll();
  Serial.println(F("Time Circuits Ready. Press D/P/L to set time."));
}

void loop() {
  unsigned long us = micros();
  if (us - tDig >= DIGIT_US) {
    tDig = us;
    showDigit(curDig);
    if (++curDig >= DIGITS_TOTAL) curDig = 0;
  }

  unsigned long ms = millis();

  if (ms - tBlink >= BLINK_MS) {
    tBlink = ms;
    blinkTick = !blinkTick;
    if (!destT.valid || !presT.valid || !lastT.valid || mode != NONE) refreshAll();
  }

  setLeds(destT, LED_AM_DEST, LED_PM_DEST, LED_SEC1_DEST, LED_SEC2_DEST, false);
  setLeds(presT, LED_AM_PRES, LED_PM_PRES, LED_SEC1_PRES, LED_SEC2_PRES, true);
  setLeds(lastT, LED_AM_LAST, LED_PM_LAST, LED_SEC1_LAST, LED_SEC2_LAST, false);

  if (presT.valid && ms - tMin >= 60000UL) {
    tMin = ms;
    presT.min++;
    if (presT.min == 60) {
      presT.min = 0;
      presT.h = (presT.h + 1) % 24;
    }
    refreshAll();
  }

  float tC = getT();
  if (tC >= 58.0 && !jumpLock && destT.valid && presT.valid) {
    lastT = presT;
    presT = destT;
    tMin = ms;
    refreshAll();
    jumpLock = true;
    Serial.println(F("Go To The Future"));
    Serial.print(F("Destination Time is: "));
    Serial.println(destT.toText());
    Serial.print(F("Present Time is: "));
    Serial.println(presT.toText());
    Serial.print(F("Last Departed Time is: "));
    Serial.println(lastT.toText());
  }
  if (tC < 34.0) jumpLock = false;

  char k = kpd.getKey();
  if (!k) return;

  if (ms - tKey < DEBOUNCE_MS && k == lastKeyPressed) return;
  tKey = ms;
  lastKeyPressed = k;

  if (mode == NONE) {
    if (k == 'D') {
      mode = SET_DEST;
      inDigits = "";
      Serial.print(F("Enter Destination Time: "));
      return;
    }
    if (k == 'P') {
      mode = SET_PRES;
      inDigits = "";
      Serial.print(F("Enter Present Time: "));
      return;
    }
    if (k == 'L') {
      mode = SET_LAST;
      inDigits = "";
      Serial.print(F("Enter Last Time Departed: "));
      return;
    }
    if (k == '#') {
      inDigits = "";
      Serial.print(F("Temp:"));
      Serial.println(tC);
      return;
    }
    return;
  }

  if (k >= '0' && k <= '9' && inDigits.length() < 12) {
    inDigits += k;
    Serial.print(k);
    return;
  }

  if (k == '#') {
    inDigits = "";
    Serial.println(F("\nOut"));
    mode = NONE;
    return;
  }

  if (k == 'R') {
    inDigits = "";
    Serial.println(F("\nReset"));
    if (mode == SET_DEST)
      Serial.print(F("Enter Destination Time: "));
    else if (mode == SET_PRES)
      Serial.print(F("Enter Present Time: "));
    else
      Serial.print(F("Enter Last Time Departed: "));
    return;
  }

  if (k != 'E')
    return;

  if (inDigits.length() != 12) {
    Serial.println(F("\nError: Need 12 digits"));
    mode = NONE;
    return;
  }

  DateTime dt = parse12(inDigits);
  if (!dt.valid) {
    Serial.println(F("\nError: Invalid date"));
    mode = NONE;
    return;
  }

  if (mode == SET_DEST) {
    destT = dt;
    Serial.print(F("\r\nThe destination date has been set: "));
  } 
  else if (mode == SET_PRES) {
    presT = dt;
    tMin = ms;
    Serial.print(F("\r\nThe present date has been set: "));
  }
  else {
    lastT = dt;
    Serial.print(F("\r\nThe last date has been set: "));
  }

  refreshAll();
  mode = NONE;
  Serial.println(dt.toText());
  // Serial.println(F("\nOK"));
}