/*****************************************************************
  Time-Circuits 15×7-Segment (Arduino Mega 2560 + 8×74HC595)

  Destination Time  : RED    (sevseg1-5)
  Present Time      : GREEN  (sevseg6-10)  
  Last Time Departed: YELLOW (sevseg11,12,13,14,15)

  Согласно ТЗ Last Time имеет особый порядок:
  - sevseg12: месяц (2 digits)
  - sevseg11: день (4 digits)
  - sevseg13: год (4 digits)
  - sevseg14: часы (2 digits)
  - sevseg15: минуты (2 digits)
*****************************************************************/

#include <Arduino.h>
#include <Keypad.h>

/* === Аппаратные пины ====================================== */
#define DATA_PIN      22
#define LATCH_PIN     23
#define CLOCK_PIN     24

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

#define NTC_PIN       A0

/* === Клавиатура 4×4 ======================================= */
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

/* === Параметры мультиплексора ============================= */
#define SR_CNT        8
#define DIGITS_TOTAL  42
#define DIGIT_US      1500UL
#define BLINK_MS      1000UL
// #define DIGIT_US      3500UL
// #define BLINK_MS      3000UL
#define DEBOUNCE_MS   200UL

/* Сегментные маски (общий катод) */
enum {D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D_MINUS,D_BLANK};
static const byte SEG_PAT[12] = {
  0b00111111, 0b00000110, 0b01011011, 0b01001111,
  0b01100110, 0b01101101, 0b01111101, 0b00000111,
  0b01111111, 0b01101111, 0b01000000, 0b00000000
};

/* === Структура времени ==================================== */
struct DateTime {
  int m, d, y, h, min; 
  bool valid;
  DateTime() { m=d=y=h=min=0; valid=false; }
};

DateTime destT, presT, lastT;

/* === Глобальные =========================================== */
byte buf[DIGITS_TOTAL];
byte curDig = 0;
unsigned long tDig=0, tBlink=0, tKey=0, tMin=0;
bool blink = false;
bool jumpLock = false;

enum Mode {NONE, SET_DEST, SET_PRES, SET_LAST};
Mode mode = NONE; 
String in;

char lastKeyPressed = '\0';

/* === Утилиты ============================================== */
bool leap(int y) { 
  return (y%4==0 && y%100!=0) || (y%400==0); 
}

bool dateOk(int M, int D, int Y, int h, int m) {
  if(M<1 || M>12 || D<1 || h>23 || m>59 || Y<0 || Y>9999) 
    return false;
  int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if(M==2 && leap(Y)) dim[1] = 29; 
  return D <= dim[M-1];
}

DateTime parse(const String& s) {
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

/* === 74HC595 ============================================== */
void latch(const byte* d) {
  digitalWrite(LATCH_PIN, LOW);
  for(int8_t i=SR_CNT-1; i>=0; --i) 
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, d[i]);
  digitalWrite(LATCH_PIN, HIGH);
}

void show(byte idx) {
  static const byte CLR[SR_CNT] = {0};
  byte sr[SR_CNT] = {0};
  sr[0] = SEG_PAT[buf[idx]];
  byte s = 1 + idx/8, b = idx%8; 
  sr[s] = 1 << b;
  latch(sr);
  delayMicroseconds(DIGIT_US);
  latch(CLR);
}

/* === Преобразование DateTime -> buf[] ==================== 
  Карта буфера:
  sevseg1:  buf[0-3]   (4 digits) - Dest Month
  sevseg2:  buf[4-5]   (2 digits) - Dest Day
  sevseg3:  buf[6-9]   (4 digits) - Dest Year
  sevseg4:  buf[10-11] (2 digits) - Dest Hour
  sevseg5:  buf[12-13] (2 digits) - Dest Min
  sevseg6:  buf[14-17] (4 digits) - Pres Month
  sevseg7:  buf[18-19] (2 digits) - Pres Day
  sevseg8:  buf[20-23] (4 digits) - Pres Year
  sevseg9:  buf[24-25] (2 digits) - Pres Hour
  sevseg10: buf[26-27] (2 digits) - Pres Min
  sevseg11: buf[28-31] (4 digits) - Last Day (!)
  sevseg12: buf[32-33] (2 digits) - Last Month (!)
  sevseg13: buf[34-37] (4 digits) - Last Year
  sevseg14: buf[38-39] (2 digits) - Last Hour
  sevseg15: buf[40-41] (2 digits) - Last Min
*/

void put2(byte off, int val) { 
  buf[off]   = val/10; 
  buf[off+1] = val%10; 
}

void put4(byte off, int val) {
  buf[off]   = (val/1000)%10;
  buf[off+1] = (val/100)%10;
  buf[off+2] = (val/10)%10;
  buf[off+3] = val%10;
}

void fillDash(byte from, byte cnt) {
  for(byte i=0; i<cnt; i++) 
    buf[from+i] = D_MINUS;
}

void fillBlank(byte from, byte cnt) {
  for(byte i=0; i<cnt; i++) 
    buf[from+i] = D_BLANK;
}

void toBufferDest(const DateTime& dt) {
  if(!dt.valid) {
    fillDash(0, 14);
    return;
  }
  // Month: 4 digits, показываем последние 2
  fillBlank(0, 2);
  put2(2, dt.m);
  // Day: 2 digits
  put2(4, dt.d);
  // Year: 4 digits
  put4(6, dt.y);
  // Hour: 2 digits (12-hour format)
  int h12; bool pm;
  to12h(dt.h, h12, pm);
  put2(10, h12);
  // Minutes: 2 digits
  put2(12, dt.min);
}

void toBufferPres(const DateTime& dt) {
  if(!dt.valid) {
    fillDash(14, 14);
    return;
  }
  // Month: 4 digits, показываем последние 2
  fillBlank(14, 2);
  put2(16, dt.m);
  // Day: 2 digits
  put2(18, dt.d);
  // Year: 4 digits
  put4(20, dt.y);
  // Hour: 2 digits (12-hour format)
  int h12; bool pm;
  to12h(dt.h, h12, pm);
  put2(24, h12);
  // Minutes: 2 digits
  put2(26, dt.min);
}

void toBufferLast(const DateTime& dt) {
  if(!dt.valid) {
    fillDash(28, 14);
    return;
  }
  // Особый порядок для Last Time!
  // sevseg11 (Day): buf[28-31] (4 digits)
  fillBlank(28, 2);
  put2(30, dt.d);

  // sevseg12 (Month): buf[32-33] (2 digits)
  put2(32, dt.m);

  // sevseg13 (Year): buf[34-37] (4 digits)
  put4(34, dt.y);

  // sevseg14 (Hour): buf[38-39] (2 digits)
  int h12; bool pm;
  to12h(dt.h, h12, pm);
  put2(38, h12);

  // sevseg15 (Min): buf[40-41] (2 digits)
  put2(40, dt.min);
}

void refresh() {
  toBufferDest(destT);
  toBufferPres(presT);
  toBufferLast(lastT);
}

/* === Управление LED ======================================= */
void leds(const DateTime& dt, int pinAM, int pinPM, int pinS1, int pinS2, bool blinkSec) {
  if(!dt.valid) {
    digitalWrite(pinAM, LOW);
    digitalWrite(pinPM, LOW);
    digitalWrite(pinS1, LOW);
    digitalWrite(pinS2, LOW);
    return;
  }

  bool pm = (dt.h >= 12);
  digitalWrite(pinAM, pm ? LOW : HIGH);
  digitalWrite(pinPM, pm ? HIGH : LOW);

  if(blinkSec) {
    // Present Time: LED моргают каждую секунду
    digitalWrite(pinS1, blink ? HIGH : LOW);
    digitalWrite(pinS2, blink ? HIGH : LOW);
  } else {
    // Destination/Last Time: LED горят постоянно
    digitalWrite(pinS1, HIGH);
    digitalWrite(pinS2, HIGH);
  }
}

/* === Чтение температуры с NTC ============================= */
float getT() {
  int raw = analogRead(NTC_PIN);
  // Простая формула для NTC Wokwi
  float voltage = raw * (5.0 / 1023.0);
  // Приблизительная линейная формула
  float tempC = (voltage - 2.5) * 20.0 + 24.0;
  return tempC;
}

/* === Setup & Loop ========================================= */
void setup() {
  Serial.begin(115200);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  pinMode(LED_AM_DEST, OUTPUT);
  pinMode(LED_PM_DEST, OUTPUT);
  pinMode(LED_SEC1_DEST, OUTPUT);
  pinMode(LED_SEC2_DEST, OUTPUT);

  pinMode(LED_AM_PRES, OUTPUT);
  pinMode(LED_PM_PRES, OUTPUT);
  pinMode(LED_SEC1_PRES, OUTPUT);
  pinMode(LED_SEC2_PRES, OUTPUT);

  pinMode(LED_AM_LAST, OUTPUT);
  pinMode(LED_PM_LAST, OUTPUT);
  pinMode(LED_SEC1_LAST, OUTPUT);
  pinMode(LED_SEC2_LAST, OUTPUT);

  refresh();
  Serial.println(F("Time Circuits Ready"));
  Serial.println(F("Press D/P/L to set time"));
}

void loop() {
  // Мультиплексирование дисплеев
  unsigned long us = micros();
  if(us - tDig >= DIGIT_US) {
    tDig = us;
    show(curDig);
    if(++curDig >= DIGITS_TOTAL) curDig = 0;
  }

  unsigned long ms = millis();

  // Моргание каждую секунду
  if(ms - tBlink >= BLINK_MS) {
    tBlink = ms;
    blink = !blink;

    // Обновляем дисплей если есть невалидные данные (для эффекта моргания дефисов)
    if(!destT.valid || !presT.valid || !lastT.valid || mode != NONE) {
      refresh();
    }
  }

  // Обновление LED
  leds(destT, LED_AM_DEST, LED_PM_DEST, LED_SEC1_DEST, LED_SEC2_DEST, false);
  leds(presT, LED_AM_PRES, LED_PM_PRES, LED_SEC1_PRES, LED_SEC2_PRES, true);
  leds(lastT, LED_AM_LAST, LED_PM_LAST, LED_SEC1_LAST, LED_SEC2_LAST, false);

  // Автоинкремент минут Present Time
  if(presT.valid && ms - tMin >= 60000UL) {
    tMin = ms; 
    presT.min++; 
    if(presT.min == 60) {
      presT.min = 0;
      presT.h = (presT.h + 1) % 24;
    }
    refresh();
  }

  // Проверка температуры для "перемещения во времени"
  float tC = getT();
  if(tC >= 34.0 && !jumpLock && destT.valid && presT.valid) {
    lastT = presT; 
    presT = destT; 
    tMin = ms; // Сбрасываем таймер для нового времени
    refresh(); 
    jumpLock = true; 
    Serial.println(F("Go To The Future"));
  }
  if(tC < 34.0) jumpLock = false;

  // Обработка клавиатуры
  char k = kpd.getKey();
  if(!k) return;

  // Защита от дребезга
  if(ms - tKey < DEBOUNCE_MS && k == lastKeyPressed) return;
  tKey = ms;
  lastKeyPressed = k;

  // Если не в режиме ввода - проверяем команды D, P, L
  if(mode == NONE) {
    if(k == 'D') {
      mode = SET_DEST; 
      in = "";
      Serial.print(F("Enter Destination Time: "));
      return;
    }
    if(k == 'P') {
      mode = SET_PRES; 
      in = "";
      Serial.print(F("Enter Present Time: "));
      return;
    }
    if(k == 'L') {
      mode = SET_LAST; 
      in = "";
      Serial.print(F("Enter Last Time Departed: "));
      return;
    }
    return;
  }

  // В режиме ввода
  if(k >= '0' && k <= '9' && in.length() < 12) {
    in += k;
    Serial.print(k);
    return;
  }

  if(k == 'R') {
    in = "";
    Serial.println(F("\nReset"));
    // Печатаем заново приглашение
    if(mode == SET_DEST) Serial.print(F("Enter Destination Time: "));
    else if(mode == SET_PRES) Serial.print(F("Enter Present Time: "));
    else Serial.print(F("Enter Last Time Departed: "));
    return;
  }

  if(k != 'E') return;

  // Нажата кнопка E - попытка установить дату
  if(in.length() != 12) {
    Serial.println(F("\nError: Need 12 digits"));
    mode = NONE;
    return;
  }

  DateTime dt = parse(in);
  if(!dt.valid) {
    Serial.println(F("\nError: Invalid date"));
    mode = NONE;
    return;
  }

  // Устанавливаем дату
  if(mode == SET_DEST) destT = dt; 
  else if(mode == SET_PRES) {
    presT = dt;
    tMin = ms; // Сбрасываем таймер минут
  }
  else lastT = dt;

  refresh(); 
  mode = NONE; 
  Serial.println(F("\nOK"));
}
