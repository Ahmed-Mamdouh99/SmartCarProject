// Controller.ino
/*
 * This is the sketch for the car controller
 * Using an Arduino Uno, 1602 LCD display, Funduino V1.0 Arduino shield and HC-05 Bluetooth module.
 * This sketch uses Arduino_FreeRTOS library and LiquidCrystal_I2C library
 * The sketch reades input from the joystick and sends the information to the car. 
 * And reads dashboard info from the car and displays it on the LCD display
 */

/* Core libraries */
#include <Wire.h>
/* Extra libraries */
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>
/* IO */
// Joystick X axis
#define PIN_X A0
// Joystick Y axis
#define PIN_Y A1
// horn button
#define PIN_HORN 4
#define PIN_ENGINE_BTN 5
/* Values */
// Message start signal for bt communication
#define START_SIG 255
// Default stack depth
#define BEEG_STACK_DEPTH 200
#define SMOL_STACK_DEPTH 90
// Task priorities
#define PRIORITY_HIGH 3
#define PRIORITY_MID 2
#define PRIORITY_LOW 1
#define PRIORITY_IDLE 0
// Task periods
// Threshold for middle value since the joystick isn't very accurate
#define ANALOG_THRESHOLD 512
// Bluetooth module baudrate
#define BLUETOOTH_BAUDRATE 9600
// LCD values
#define LCD_ADDR 0x27
#define LCD_WIDTH 16
#define LCD_HEIGHT 2
// LCD custom characters macros
#define CHR_LCK_ON 0
#define CHR_LCK_OFF 1
// Bit vales for flags byte in lcdInfo struct
#define FLAG_LOCK (_BV(1)) // Car lock flag bit for lcd info
// Bit values for flags byte in controller info struct
#define FLAG_L_POS (_BV(0))
#define FLAG_R_POS (_BV(1))
#define FLAG_HORN (_BV(2)) // Horn flag bit for control signal
#define FLAG_ENGINE (_BV(3))
#define FLAG_FORWARDS (_BV(4))
// Gears
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_D 2
#define GEAR_N 3
/* Functions */
#define bitAssign(data, bitValue, check) data = (check) ? (data | bitValue) : (data & ~(bitValue))

typedef struct
{
  byte r_spd;
  byte l_spd;
  byte flags;
} ctrlInfo_t;

typedef struct
{
  /* This struct contains information to be displayed on the lcd */
  byte day;    // (0-99)
  byte month;  // (0-99)
  byte year;   // (0-99)
  byte hour;   // (0-99)
  byte minute; // (0-99)
  byte temp;   // Temperature reading (0-99)
  byte flags;  // lock flags
} lcdInfo_t;

SemaphoreHandle_t semaphore_lock;
SemaphoreHandle_t semaphore_gear;
byte gear;
SemaphoreHandle_t semaphore_engine;

void setup()
{
#define ENGINE_MAX 2
#define LOCK_MAX 2
  // Start Serial
  Serial.begin(BLUETOOTH_BAUDRATE);
  // Set IO
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_HORN, INPUT_PULLUP);
  pinMode(PIN_ENGINE_BTN, INPUT_PULLUP);
  // Initialize semaphores
  semaphore_lock = xSemaphoreCreateCounting(LOCK_MAX, 0);
  semaphore_gear = xSemaphoreCreateMutex();
  semaphore_engine = xSemaphoreCreateCounting(ENGINE_MAX, 0);
  // Create tasks
  xTaskCreate(TaskRead, (const char *)"read", BEEG_STACK_DEPTH, NULL, PRIORITY_IDLE, NULL);
  xTaskCreate(TaskSend, (const char *)"send", SMOL_STACK_DEPTH, NULL, PRIORITY_HIGH, NULL);
  xTaskCreate(TaskEngineCheck, (const char *)"eng_chk", SMOL_STACK_DEPTH, NULL, PRIORITY_HIGH, NULL);
  vTaskStartScheduler();
}

void TaskRead(void *params)
{
#define PERIOD_READ (pdMS_TO_TICKS(200))
  /* Continuous task */
  lcdInfo_t info;
  byte *buff_ptr = (byte *)&info;
  byte *buff_max = (byte *)&info + sizeof(lcdInfo_t);
  byte data;
  // Initialize lcd
  LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("LOCKED");
  // State management
  bool locked = true;
  bool engine_off = true;
  // Main loop
  while (1)
  {
    if (Serial.available() > 0)
    {
      data = Serial.read();
      if (data == START_SIG)
      {
        // Reset buffer position
        buff_ptr = (byte *)&info;
      }
      else if (buff_ptr < buff_max)
      {
        *buff_ptr = data;
        buff_ptr++;
      }
      // Else drop byte
    }
    if (buff_ptr == buff_max)
    {
      // Buffer full
      lcd.setCursor(0, 0);
      if (info.flags & FLAG_LOCK)
      {
        // Take semaphore
        if (!locked)
        {
          for (int i = 0; i < LOCK_MAX; ++i)
          {
            xSemaphoreTake(semaphore_lock, portMAX_DELAY);
          }
          engine_off = false;
          locked = true;
          lcd.clear();
          lcd.print("LOCKED");
        }
      }
      else
      {
        if (locked)
        {
          // Free semaphore
          for (int i = 0; i < LOCK_MAX; ++i)
          {
            xSemaphoreGive(semaphore_lock);
          }
          locked = false;
        }
        if (xSemaphoreTake(semaphore_engine, 0) == pdTRUE)
        {
          engine_off = false;
          // Display info on LCD
          lcd.print((info.day / 10) % 10);
          lcd.print(info.day % 10);
          lcd.print('/');
          lcd.print((info.month / 10) % 10);
          lcd.print(info.month % 10);
          lcd.print('/');
          lcd.print((info.year / 10) % 10);
          lcd.print(info.year % 10);
          lcd.print(' ');
          lcd.print((info.hour / 10) % 10);
          lcd.print(info.hour % 10);
          lcd.print(':');
          lcd.print((info.minute / 10) % 10);
          lcd.print(info.minute % 10);
          lcd.setCursor(0, 1);
          lcd.print((info.temp / 10) % 10);
          lcd.print(info.temp % 10);
          lcd.print("C ");
          if (xSemaphoreTake(semaphore_gear, portMAX_DELAY) == pdTRUE)
          {
            char gear_char;
            switch(gear)
            {
              case(GEAR_P): gear_char = 'P'; break;
              case(GEAR_R): gear_char = 'r'; break;
              case(GEAR_N): gear_char = 'N'; break;
              case(GEAR_D): gear_char = 'D'; break;
            }
            lcd.write(gear_char);
            xSemaphoreGive(semaphore_gear);
          }
          // Free semaphore
          xSemaphoreGive(semaphore_engine);
        }
        else
        {
          if (!engine_off)
          {
            lcd.clear();
            lcd.print("ENGINE OFF");
            engine_off = true;
          }
        }
      }
    }
  }
}

void TaskSend(void *params)
{
#define PERIOD_SEND (pdMS_TO_TICKS(100))
#define THRESHOLD_Y (-5)
#define THRESHOLD_X (-5)
#define WITHIN(x, l, h) (max(l, min(x, h)))
  ctrlInfo_t info;
  int y;
  int x;
  // State management
  bool forwards = true;
  TickType_t prev_wakeup_time = xTaskGetTickCount();
  while (1)
  {
    if (xSemaphoreTake(semaphore_lock, portMAX_DELAY) == pdTRUE)
    {
      // Read input
      info.flags = 0;
      // Read joystick input
      y = map(WITHIN(analogRead(PIN_Y) - THRESHOLD_Y, 0, 1024), 0, 1024, -254, 254);
      x = map(WITHIN(analogRead(PIN_X) - THRESHOLD_X, 0, 1024), 0, 1024, -254, 254);
      // Determine the gear
      if(xSemaphoreTake(semaphore_gear, portMAX_DELAY) == pdTRUE)
      {
        if(abs(x) > 10)
        {
          gear = GEAR_N;
        }
        else if(abs(y) < 10)
        {
          gear = GEAR_P;
        }
        else if(y > 0)
        {
          gear = GEAR_D;
        }
        else
        {
          gear = GEAR_R;
        }
        xSemaphoreGive(semaphore_gear);
      }
      // set forwards flag
      bitAssign(info.flags, FLAG_FORWARDS, y > 0);
      // set horn flag
      bitAssign(info.flags, FLAG_HORN, digitalRead(PIN_HORN) == LOW);
      // Set the engine on flag
      bitAssign(info.flags, FLAG_ENGINE, false);
      if (xSemaphoreTake(semaphore_engine, (TickType_t)0) == pdTRUE)
      {
        bitAssign(info.flags, FLAG_ENGINE, true);
        // Free semaphore
        xSemaphoreGive(semaphore_engine);
      }
      if (y >= 0)
      {
        bitAssign(info.flags, FLAG_R_POS, true);
        bitAssign(info.flags, FLAG_L_POS, true);
        info.l_spd = WITHIN(y + x, 0, 254);
        info.r_spd = WITHIN(y - x, 0, 254);
      }
      else
      {
        y = -y;
        bitAssign(info.flags, FLAG_R_POS, false);
        bitAssign(info.flags, FLAG_L_POS, false);
        info.l_spd = WITHIN(y + x, 0, 254);
        info.r_spd = WITHIN(y - x, 0, 254);
      }
      // Calculate right and left speeds
      if (info.l_spd < 10)
      {
        info.l_spd = 0;
      }
      if (info.r_spd < 10)
      {
        info.r_spd = 0;
      }
      // Send info to car
      // Send start sig
      Serial.write(START_SIG);
      for (int i = 0; i < sizeof(ctrlInfo_t); ++i)
      {
        Serial.write(((byte *)&info)[i]);
      }
      // Free semaphore
      xSemaphoreGive(semaphore_lock);
    }
    vTaskDelayUntil(&prev_wakeup_time, PERIOD_SEND);
  }
}

void TaskEngineCheck(void *params)
{
#define PERIOD_ENGINE_CHECK (pdMS_TO_TICKS(100))
  bool engine_on = false;
  bool btn_dwn = false;
  TickType_t prev_wakeup_time = xTaskGetTickCount();
  // Main loop
  while (1)
  {
    if (xSemaphoreTake(semaphore_lock, portMAX_DELAY) == pdTRUE)
    {
      if (digitalRead(PIN_ENGINE_BTN) == HIGH)
      {
        // Button not pressed
        if (btn_dwn)
        {
          btn_dwn = false;
        }
      }
      else
      {
        // Button pressed
        if (!btn_dwn)
        {
          btn_dwn = true;
          // Switch engine state
          if (engine_on)
          {
            for (int i = 0; i < ENGINE_MAX; ++i)
            {
              xSemaphoreTake(semaphore_engine, portMAX_DELAY);
            }
          }
          else
          {
            for (int i = 0; i < ENGINE_MAX; ++i)
            {
              xSemaphoreGive(semaphore_engine);
            }
          }
          engine_on = !engine_on;
        }
      }
      // Free semaphore
      xSemaphoreGive(semaphore_lock);
    }
    // Sleep until next period
    vTaskDelayUntil(&prev_wakeup_time, PERIOD_ENGINE_CHECK);
  }
}

void loop() {}
