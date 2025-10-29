#include <Arduino.h>
#include <Keypad.h>

// ============= КОНФИГУРАЦИЯ ПИНОВ =============

// Shift Registers (74HC595)
#define DATA_PIN  22   // DS (Serial Data)
#define LATCH_PIN 23   // STCP (Storage Register Clock / Latch)
#define CLOCK_PIN 24   // SHCP (Shift Register Clock)

// LED индикаторы
#define LED_AM_DEST    30
#define LED_PM_DEST    31
#define LED_SEC1_DEST  32
#define LED_SEC2_DEST  33

#define LED_AM_PRES    34
#define LED_PM_PRES    35
#define LED_SEC1_PRES  36
#define LED_SEC2_PRES  37

#define LED_AM_LAST    38
#define LED_PM_LAST    39
#define LED_SEC1_LAST  40
#define LED_SEC2_LAST  41

// NTC датчик температуры
#define NTC_PIN A0

// Keypad конфигурация
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'D'},
  {'4', '5', '6', 'P'},
  {'7', '8', '9', 'L'},
  {'R', '0', '#', 'E'}
};
byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, 7, 8, 9};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ============= СТРУКТУРЫ ДАННЫХ =============

struct DateTime {
  int month;   // 1-12
  int day;     // 1-31
  int year;    // 0-9999
  int hour;    // 0-23 (внутреннее хранение)
  int minute;  // 0-59
  bool isSet;
  bool isValid;
};

DateTime destinationTime = {0, 0, 0, 0, 0, false, false};
DateTime presentTime = {0, 0, 0, 0, 0, false, false};
DateTime lastTime = {0, 0, 0, 0, 0, false, false};

// ============= ПЕРЕМЕННЫЕ =============

String inputBuffer = "";
enum InputMode { NONE, DEST_INPUT, PRES_INPUT, LAST_INPUT };
InputMode currentInputMode = NONE;

unsigned long lastBlinkTime = 0;
bool blinkState = false;
const unsigned long BLINK_INTERVAL = 1000; // 1 секунда

unsigned long lastSecondUpdate = 0;
unsigned long lastKeyPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 200; // 200ms для защиты от дребезга

float lastTemperature = 24.0;
bool timeJumpTriggered = false;

// ============= 7-SEGMENT PATTERNS =============
// Паттерны для отображения цифр на 7-сегментном дисплее
const byte digitPatterns[11] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b01000000  // - (минус для "----")
};

// ============= ФУНКЦИИ =============

// Функция валидации даты
bool isValidDate(int month, int day, int year, int hour, int minute) {
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (year < 0 || year > 9999) return false;
  if (hour < 0 || hour > 23) return false;
  if (minute < 0 || minute > 59) return false;
  
  // Проверка дней в месяце
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  
  // Проверка високосного года
  if (month == 2) {
    bool isLeap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    if (isLeap) daysInMonth[1] = 29;
  }
  
  if (day > daysInMonth[month - 1]) return false;
  
  return true;
}

// Парсинг введенной строки (12 цифр: ММДДYYYYHHMM)
DateTime parseInput(String input) {
  DateTime dt;
  dt.isSet = false;
  dt.isValid = false;
  
  if (input.length() != 12) return dt;
  
  dt.month = input.substring(0, 2).toInt();
  dt.day = input.substring(2, 4).toInt();
  dt.year = input.substring(4, 8).toInt();
  dt.hour = input.substring(8, 10).toInt();
  dt.minute = input.substring(10, 12).toInt();
  
  dt.isValid = isValidDate(dt.month, dt.day, dt.year, dt.hour, dt.minute);
  dt.isSet = dt.isValid;
  
  return dt;
}

// Преобразование 24-часового формата в 12-часовой
void get12HourFormat(int hour24, int& hour12, bool& isPM) {
  isPM = hour24 >= 12;
  hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;
}

// Отправка данных в shift registers
void updateShiftRegisters(byte* data, int numBytes) {
  digitalWrite(LATCH_PIN, LOW);
  for (int i = numBytes - 1; i >= 0; i--) {
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data[i]);
  }
  digitalWrite(LATCH_PIN, HIGH);
}

// Обновление всех дисплеев через shift registers
void updateAllDisplays() {
  // Здесь должна быть логика обновления всех дисплеев
  // Для упрощения в Wokwi будем использовать прямое управление сегментами
  // В реальном проекте нужно мультиплексирование
  
  // Пока заглушка - в полной версии здесь будет управление всеми 15 дисплеями
  byte displayData[4] = {0, 0, 0, 0};
  updateShiftRegisters(displayData, 4);
}

// Обновление LED индикаторов для Destination Time
void updateDestinationLEDs() {
  if (!destinationTime.isSet || !destinationTime.isValid) {
    digitalWrite(LED_AM_DEST, LOW);
    digitalWrite(LED_PM_DEST, LOW);
    digitalWrite(LED_SEC1_DEST, blinkState ? LOW : HIGH);
    digitalWrite(LED_SEC2_DEST, blinkState ? LOW : HIGH);
    return;
  }
  
  int hour12;
  bool isPM;
  get12HourFormat(destinationTime.hour, hour12, isPM);
  
  digitalWrite(LED_AM_DEST, !isPM ? HIGH : LOW);
  digitalWrite(LED_PM_DEST, isPM ? HIGH : LOW);
  digitalWrite(LED_SEC1_DEST, HIGH);
  digitalWrite(LED_SEC2_DEST, HIGH);
}

// Обновление LED индикаторов для Present Time
void updatePresentLEDs() {
  if (!presentTime.isSet || !presentTime.isValid) {
    digitalWrite(LED_AM_PRES, LOW);
    digitalWrite(LED_PM_PRES, LOW);
    digitalWrite(LED_SEC1_PRES, LOW);
    digitalWrite(LED_SEC2_PRES, LOW);
    return;
  }
  
  int hour12;
  bool isPM;
  get12HourFormat(presentTime.hour, hour12, isPM);
  
  digitalWrite(LED_AM_PRES, !isPM ? HIGH : LOW);
  digitalWrite(LED_PM_PRES, isPM ? HIGH : LOW);
  digitalWrite(LED_SEC1_PRES, blinkState ? HIGH : LOW);
  digitalWrite(LED_SEC2_PRES, blinkState ? HIGH : LOW);
}

// Обновление LED индикаторов для Last Time
void updateLastLEDs() {
  if (!lastTime.isSet || !lastTime.isValid) {
    digitalWrite(LED_AM_LAST, LOW);
    digitalWrite(LED_PM_LAST, LOW);
    digitalWrite(LED_SEC1_LAST, blinkState ? LOW : HIGH);
    digitalWrite(LED_SEC2_LAST, blinkState ? LOW : HIGH);
    return;
  }
  
  int hour12;
  bool isPM;
  get12HourFormat(lastTime.hour, hour12, isPM);
  
  digitalWrite(LED_AM_LAST, !isPM ? HIGH : LOW);
  digitalWrite(LED_PM_LAST, isPM ? HIGH : LOW);
  digitalWrite(LED_SEC1_LAST, HIGH);
  digitalWrite(LED_SEC2_LAST, HIGH);
}

// Вывод времени в Serial для отладки
void printDateTime(const char* label, DateTime& dt) {
  Serial.print(label);
  if (!dt.isSet || !dt.isValid) {
    Serial.println("-- -- ---- --:--");
    return;
  }
  
  int hour12;
  bool isPM;
  get12HourFormat(dt.hour, hour12, isPM);
  
  if (dt.month < 10) Serial.print("0");
  Serial.print(dt.month);
  Serial.print(" ");
  
  if (dt.day < 10) Serial.print("0");
  Serial.print(dt.day);
  Serial.print(" ");
  
  Serial.print(dt.year);
  Serial.print(" ");
  
  if (hour12 < 10) Serial.print("0");
  Serial.print(hour12);
  Serial.print(":");
  
  if (dt.minute < 10) Serial.print("0");
  Serial.print(dt.minute);
  Serial.print(" ");
  Serial.println(isPM ? "PM" : "AM");
}

// Функция "перемещения во времени"
void performTimeJump() {
  if (!destinationTime.isSet || !presentTime.isSet) return;
  
  Serial.println("\n=== TIME TRAVEL INITIATED ===");
  Serial.println("Temperature reached 34°C!");
  
  // Last Time = Current Present Time
  lastTime = presentTime;
  
  // Present Time = Destination Time
  presentTime = destinationTime;
  
  Serial.println("=== TIME TRAVEL COMPLETE ===");
  printDateTime("Destination Time: ", destinationTime);
  printDateTime("Present Time:     ", presentTime);
  printDateTime("Last Time:        ", lastTime);
  Serial.println();
  
  timeJumpTriggered = true;
}

// Чтение температуры
float readTemperature() {
  int reading = analogRead(NTC_PIN);
  // Простое преобразование для NTC термистора
  // В реальности нужна более сложная формула Стейнхарта-Харта
  float voltage = reading * (5.0 / 1023.0);
  float resistance = (10000.0 * voltage) / (5.0 - voltage);
  
  // Упрощенная линейная аппроксимация для демонстрации
  float temperature = 25.0 + (voltage - 2.5) * 10.0;
  
  return temperature;
}

// ============= SETUP =============

void setup() {
  Serial.begin(115200);
  Serial.println("=================================");
  Serial.println("TIME CIRCUITS - Back to the Future");
  Serial.println("=================================\n");
  
  // Настройка пинов Shift Registers
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  
  // Настройка LED пинов
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
  
  // Настройка NTC
  pinMode(NTC_PIN, INPUT);
  
  // Начальная инициализация дисплеев
  updateAllDisplays();
  
  Serial.println("System initialized. Ready to set dates.");
  Serial.println("Press D for Destination Time");
  Serial.println("Press P for Present Time");
  Serial.println("Press L for Last Time Departed\n");
}

// ============= LOOP =============

void loop() {
  unsigned long currentMillis = millis();
  
  // Обновление мигания каждую секунду
  if (currentMillis - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = currentMillis;
    blinkState = !blinkState;
    
    // Обновление Present Time если установлено
    if (presentTime.isSet && presentTime.isValid) {
      presentTime.minute++;
      if (presentTime.minute >= 60) {
        presentTime.minute = 0;
        presentTime.hour++;
        if (presentTime.hour >= 24) {
          presentTime.hour = 0;
          presentTime.day++;
          // Упрощенно, без учета месяцев и годов
        }
      }
    }
  }
  
  // Обновление LED индикаторов
  updateDestinationLEDs();
  updatePresentLEDs();
  updateLastLEDs();
  
  // Чтение температуры
  float temperature = readTemperature();
  if (temperature >= 34.0 && !timeJumpTriggered) {
    performTimeJump();
  } else if (temperature < 34.0) {
    timeJumpTriggered = false; // Сброс для следующего прыжка
  }
  
  // Обработка клавиатуры
  char key = keypad.getKey();
  
  if (key && (currentMillis - lastKeyPressTime > DEBOUNCE_DELAY)) {
    lastKeyPressTime = currentMillis;
    
    // Если ожидается ввод
    if (currentInputMode != NONE) {
      if (key >= '0' && key <= '9') {
        if (inputBuffer.length() < 12) {
          inputBuffer += key;
          Serial.print(key);
        }
      } else if (key == 'R') {
        // Сброс ввода
        inputBuffer = "";
        Serial.println("\nInput reset.");
        
        if (currentInputMode == DEST_INPUT) {
          Serial.print("Enter Destination Time: ");
        } else if (currentInputMode == PRES_INPUT) {
          Serial.print("Enter Present Time: ");
        } else if (currentInputMode == LAST_INPUT) {
          Serial.print("Enter Last Time Departed: ");
        }
      } else if (key == 'E') {
        // Подтверждение ввода
        if (inputBuffer.length() == 12) {
          DateTime parsedDate = parseInput(inputBuffer);
          
          if (parsedDate.isValid) {
            if (currentInputMode == DEST_INPUT) {
              destinationTime = parsedDate;
              Serial.println("\nDestination Time set successfully!");
              printDateTime("Destination Time: ", destinationTime);
            } else if (currentInputMode == PRES_INPUT) {
              presentTime = parsedDate;
              Serial.println("\nPresent Time set successfully!");
              printDateTime("Present Time: ", presentTime);
            } else if (currentInputMode == LAST_INPUT) {
              lastTime = parsedDate;
              Serial.println("\nLast Time Departed set successfully!");
              printDateTime("Last Time: ", lastTime);
            }
          } else {
            Serial.println("\nInvalid date! Please try again.");
          }
          
          inputBuffer = "";
          currentInputMode = NONE;
        } else {
          Serial.println("\nIncomplete input! Need 12 digits (MMDDYYYYHHMM)");
        }
      }
    } else {
      // Начало нового ввода
      if (key == 'D') {
        currentInputMode = DEST_INPUT;
        inputBuffer = "";
        Serial.print("\nEnter Destination Time: ");
      } else if (key == 'P') {
        currentInputMode = PRES_INPUT;
        inputBuffer = "";
        Serial.print("\nEnter Present Time: ");
      } else if (key == 'L') {
        currentInputMode = LAST_INPUT;
        inputBuffer = "";
        Serial.print("\nEnter Last Time Departed: ");
      }
    }
  }
  
  // Небольшая задержка для стабильности
  delay(10);
}