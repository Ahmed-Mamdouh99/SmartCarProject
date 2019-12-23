// Car.ino
/* Core libraries */
#include <Wire.h>
#include <SPI.h>
/* Extra libraries */
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <DS1307RTC.h>
#include <TimeLib.h>
#include <MFRC522.h>
#include <NewPing.h>
/* IO */
#define PIN_LDR A6
#define PIN_TEMP A7
#define PIN_AEB_TRIG 25
#define PIN_AEB_ECHO 2
#define PIN_AEB_BUZZ 23
#define PIN_HEADLIGHTS 22
#define PIN_RFID_RST 5
#define PIN_RFID_SDA 53
#define PIN_CAR_HORN 24
#define PIN_MOTORS_L_EN 3
#define PIN_MOTORS_R_EN 6
#define PIN_MOTORS_L_POS 26
#define PIN_MOTORS_L_NEG 27
#define PIN_MOTORS_R_POS 28
#define PIN_MOTORS_R_NEG 29
/* Values */
// Task priorities
#define PRIORITY_HIGH 3
#define PRIORITY_MID 2
#define PRIORITY_LOW 1
#define PRIORITY_IDLE 0
// Task periods in tick type
// Message start signal for bt communication
#define START_SIG 255
// Default stack depth
#define BEEG_STACK_DEPTH 200
#define BEEEG_STACK_DEPTH 250
#define SMOL_STACK_DEPTH 150
// Bluetooth module baudrate
#define BLUETOOTH_BAUDRATE 9600
// Bit vales for flags byte in lcdInfo struct
#define FLAG_LOCK (_BV(1)) // Car lock flag bit for lcd info
// Bit values for flags byte in controller info struct
#define FLAG_L_POS (_BV(0))
#define FLAG_R_POS (_BV(1))
#define FLAG_HORN (_BV(2))     // Horn flag bit for control signal
#define FLAG_ENGINE (_BV(3))   // Engine on/off flag
#define FLAG_FORWARDS (_BV(4)) // Moving forwards or back
// Analog thresholds
#define LDR_THRESHOLD 300         // Ldr value that corresponds to a dark room
#define MOTORS_NOISE_THRESHOLD 20 // Threshold for starting motors
// Bit values
#define FORWARDS 1
#define BACKWARDS 0
/* Functions */
#define bitAssign(data, bitValue, check) data = (check) ? (data | bitValue) : (data & ~(bitValue))
#define TEMP_TO_CELSIUS(x) (x * 4.88)
#define DEBUG
#ifdef DEBUG
#define START_LOG Serial.begin(9600);
#define LOG(x) Serial.print(x);
#else
#define START_LOG
#define LOG(x)
#endif

/* Structs */
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

/* Global -variables- semaphores */
SemaphoreHandle_t semaphore_lock;
SemaphoreHandle_t semaphore_engine;
SemaphoreHandle_t semaphore_aeb;
QueueHandle_t queue_info;

void setup()
{
#define LOCK_MAX_COUNT 3
#define ENGINE_MAX_COUNT 3
    START_LOG
    LOG("UP\n")
    // Start Serial
    Serial1.begin(BLUETOOTH_BAUDRATE);
    // Setup IO
    pinMode(PIN_AEB_BUZZ, OUTPUT);
    pinMode(PIN_HEADLIGHTS, OUTPUT);
    pinMode(PIN_CAR_HORN, OUTPUT);
    // Initialize semaphores
    semaphore_lock = xSemaphoreCreateCounting(LOCK_MAX_COUNT, 0);     /* Initially locked */
    semaphore_engine = xSemaphoreCreateCounting(ENGINE_MAX_COUNT, 0); /* Initially the engine is off */
    semaphore_aeb = xSemaphoreCreateBinary();                         /* Initially AEB OFF */
    xSemaphoreGive(semaphore_aeb);
    // Initialize queue
    queue_info = xQueueCreate(1, sizeof(ctrlInfo_t));
    // Create tasks
    xTaskCreate(TaskSend, (const char *)"send", BEEG_STACK_DEPTH, NULL, PRIORITY_MID, NULL);
    xTaskCreate(TaskLockCar, (const char *)"lock", BEEG_STACK_DEPTH, NULL, PRIORITY_LOW, NULL);
    xTaskCreate(TaskControlCar, (const char *)"ctrl", SMOL_STACK_DEPTH, NULL, PRIORITY_HIGH, NULL);
    xTaskCreate(TaskAeb, (const char *)"aeb", SMOL_STACK_DEPTH, NULL, PRIORITY_HIGH, NULL);
    xTaskCreate(TaskHeadlights, (const char *)"head", SMOL_STACK_DEPTH, NULL, PRIORITY_MID, NULL);
    xTaskCreate(TaskRead, (const char *)"read", SMOL_STACK_DEPTH, NULL, PRIORITY_IDLE, NULL);
    vTaskStartScheduler();
}

void TaskSend(void *params)
{
#define PERIOD_SEND (pdMS_TO_TICKS(200)) /* Every 200 millis */
    lcdInfo_t info;
    tmElements_t tm;
    bool locked;
    TickType_t prev_wakeup_time = xTaskGetTickCount();
    // Main loop
    while (1)
    {
        // Reset flags byte
        info.flags = 0;
        // Check if the car is locked
        if (locked)
        {
            if (xSemaphoreTake(semaphore_lock, portMAX_DELAY) == pdTRUE)
            {
                locked = false;
                // Free semaphore
                xSemaphoreGive(semaphore_lock);
            }
        }
        else
        {
            if (xSemaphoreTake(semaphore_lock, (TickType_t)0) == pdTRUE)
            {
                locked = false;
                // Free semaphore
                xSemaphoreGive(semaphore_lock);
            }
            else
            {
                locked = true;
            }
        }
        // Update lock flag
        bitAssign(info.flags, FLAG_LOCK, locked);
        // Read rtc time
        if (RTC.read(tm))
        {
            info.day = tm.Day;
            info.month = tm.Month;
            info.year = tmYearToCalendar(tm.Year) - 2000;
            info.hour = tm.Hour;
            info.minute = tm.Minute;
        }
        // Update temperature reading
        info.temp = min(254, TEMP_TO_CELSIUS(analogRead(PIN_TEMP)));
        // Send info to controller
        Serial1.write(START_SIG);
        for (byte i = 0; i < sizeof(lcdInfo_t); i++)
        {
            Serial1.write(((byte *)&info)[i]);
        }
        // Sleep until next period
        vTaskDelayUntil(&prev_wakeup_time, PERIOD_SEND);
    }
}

void TaskLockCar(void *params)
{
#define PERIOD_LOCK (pdMS_TO_TICKS(200)) /* Every 200 millis */
    // Defining key uid
    const byte keyUID[4] = {0x77, 0xC2, 0x50, 0x33};
    // Initialize RFID reader and key
    MFRC522 reader(PIN_RFID_SDA, PIN_RFID_RST);
    MFRC522::MIFARE_Key key;
    SPI.begin();
    reader.PCD_Init();
    // State managing
    bool car_locked = true;
    bool match = false;
    // Initialize timestamp
    TickType_t prev_wakeup_time = xTaskGetTickCount();
    // Main loop
    while (1)
    {
        if (!reader.PICC_IsNewCardPresent())
        {
            // Sleep until next period
            vTaskDelayUntil(&prev_wakeup_time, PERIOD_LOCK);
            continue;
        }
        // Read card ID
        if (!reader.PICC_ReadCardSerial())
        {
            // Sleep until next period
            vTaskDelayUntil(&prev_wakeup_time, PERIOD_LOCK);
            continue;
        }
        // Card serial read
        match = true;
        // Compare card UID with the key
        for (byte i = 0; i < reader.uid.size; i++)
        {
            if (reader.uid.uidByte[i] != keyUID[i])
            {
                // Not the same card as the one read before
                match = false;
                break;
            }
        }
        if (match)
        {
            // If key is valid switch the lock
            if (car_locked)
            {
                for (int i = 0; i < LOCK_MAX_COUNT; ++i)
                {
                    xSemaphoreGive(semaphore_lock);
                }
                LOG("Unlocked\n")
            }
            else
            {
                for (int i = 0; i < LOCK_MAX_COUNT; ++i)
                {
                    xSemaphoreTake(semaphore_lock, portMAX_DELAY);
                }
                LOG("Locked\n")
            }
            // Switch lock state
            car_locked = !car_locked;
        }
        else
        {
            // If the key is invalid then lock the car
            for (int i = 0; i < LOCK_MAX_COUNT; ++i)
            {
                xSemaphoreTake(semaphore_lock, portMAX_DELAY);
            }
            LOG("Locked\n")
            car_locked = true;
        }
        // Else no card present
        // Sleep until next period
        vTaskDelayUntil(&prev_wakeup_time, PERIOD_LOCK);
    }
}

void TaskControlCar(void *params)
{
#define PERIOD_CONTROL (pdMS_TO_TICKS(100)) /* Every 100 millis */
    ctrlInfo_t info;
    TickType_t prev_wakeup_time = xTaskGetTickCount();
    // Main loop
    while (1)
    {
        if (xQueueReceive(queue_info, &info, portMAX_DELAY) == pdTRUE)
        {
            // Turn off all motors initially
            analogWrite(PIN_MOTORS_L_EN, 0);
            analogWrite(PIN_MOTORS_R_EN, 0);
            // Check if the engine is up
            if (xSemaphoreTake(semaphore_engine, portMAX_DELAY) == pdTRUE)
            {
                // Set motors directions
                digitalWrite(PIN_MOTORS_L_POS, info.flags & FLAG_L_POS ? HIGH : LOW);
                digitalWrite(PIN_MOTORS_L_NEG, info.flags & FLAG_L_POS ? LOW : HIGH);
                digitalWrite(PIN_MOTORS_R_POS, info.flags & FLAG_R_POS ? HIGH : LOW);
                digitalWrite(PIN_MOTORS_R_NEG, info.flags & FLAG_R_POS ? LOW : HIGH);
                if (info.flags & FLAG_FORWARDS)
                {
                    // Check if AEB is off right now
                    if (xSemaphoreTake(semaphore_aeb, (TickType_t)0) == pdTRUE)
                    {
                        // Move car
                        analogWrite(PIN_MOTORS_L_EN, info.l_spd);
                        analogWrite(PIN_MOTORS_R_EN, info.r_spd);
                        // Free semaphore
                        xSemaphoreGive(semaphore_aeb);
                    }
                    // Else do nothing
                }
                else
                {
                    // Move car
                    analogWrite(PIN_MOTORS_L_EN, info.l_spd);
                    analogWrite(PIN_MOTORS_R_EN, info.r_spd);
                }
                // Check horn
                digitalWrite(PIN_CAR_HORN, info.flags & FLAG_HORN ? HIGH : LOW);
                // Free semaphore
                xSemaphoreGive(semaphore_engine);
            }
        }
        // Sleep until next period
        vTaskDelayUntil(&prev_wakeup_time, PERIOD_CONTROL);
    }
}

void TaskAeb(void *params)
{
#define PERIOD_AEB (pdMS_TO_TICKS(50)) /* Every 50 millis */
#define AEB_MAX_DIST 40                /* 40 cm */
#define AEB_BUZZ_DIST 35               /* 35 cm */
#define AEB_BRAKE_DIST 25              /* 25 cm */
    TickType_t prev_wakeup_time = xTaskGetTickCount();
    NewPing sonar(PIN_AEB_TRIG, PIN_AEB_ECHO, AEB_MAX_DIST);
    uint8_t reading;
    // State management
    bool aeb_on = false;
    // Main loop
    while (1)
    {
        if (xSemaphoreTake(semaphore_engine, portMAX_DELAY) == pdTRUE)
        {
            // Get reading from sonar
            reading = sonar.ping_cm();
            if (reading == 0 || reading > AEB_BUZZ_DIST)
            {
                // Turn off AEB
                xSemaphoreGive(semaphore_aeb);
                // Turn off buzzer
                digitalWrite(PIN_AEB_BUZZ, LOW);
                aeb_on = false;
            }
            else if (reading > AEB_BRAKE_DIST)
            {
                // Turn off AEB
                xSemaphoreGive(semaphore_aeb);
                // Turn on buzzer
                digitalWrite(PIN_AEB_BUZZ, HIGH);
                aeb_on = false;
            }
            else
            {
                // Turn on AEB
                if (!aeb_on)
                {
                    xSemaphoreTake(semaphore_aeb, portMAX_DELAY);
                    // Turn on buzzer
                    digitalWrite(PIN_AEB_BUZZ, HIGH);
                    // Stop car
                    analogWrite(PIN_MOTORS_L_EN, 0);
                    analogWrite(PIN_MOTORS_R_EN, 0);
                    aeb_on = true;
                }
            }
            // Free engine semaphore
            xSemaphoreGive(semaphore_engine);
        }
        // Sleep until next period
        vTaskDelayUntil(&prev_wakeup_time, PERIOD_AEB);
    }
}

void TaskHeadlights(void *params)
{
#define PERIOD_HEADLIGHTS (pdMS_TO_TICKS(200)) /* Every 200 millis */
#define LDR_THRESHOLD 400                      /* threshold for the light intensity */
    TickType_t prev_wakeup_time = xTaskGetTickCount();
    // Main loop
    while (1)
    {
        if (xSemaphoreTake(semaphore_engine, portMAX_DELAY) == pdTRUE)
        {
            // Set headlights according to ldr reading
            if (analogRead(PIN_LDR) > LDR_THRESHOLD)
            {
                // Turn off headlights
                digitalWrite(PIN_HEADLIGHTS, LOW);
            }
            else
            {
                // Turn on headlights
                digitalWrite(PIN_HEADLIGHTS, HIGH);
            }
            xSemaphoreGive(semaphore_engine);
        }
        // Delay until next cycle
        vTaskDelayUntil(&prev_wakeup_time, PERIOD_HEADLIGHTS);
    }
}

void TaskRead(void *params)
{
    /* Continuous task */
    ctrlInfo_t info;
    byte *buff_ptr = (byte *)&info;
    byte *buff_max = buff_ptr + sizeof(ctrlInfo_t);
    byte data;
    // State management
    bool engine_on = false;
    // Initialize timestamp
    TickType_t timestamp = xTaskGetTickCount();
    while (1)
    {
        if (Serial1.available())
        {
            // Read new byte
            data = Serial1.read();
            if (data == START_SIG)
            {
                // Restart buffer
                buff_ptr = (byte *)&info;
            }
            else if (buff_ptr < buff_max)
            {
                // Buffer input
                *buff_ptr = data;
                buff_ptr++;
            }
            // Else drop data
        }
        if (buff_ptr == buff_max)
        {
            // Copy buffer to queue
            xQueueSend(queue_info, &info, (TickType_t)0);
            if (xSemaphoreTake(semaphore_lock, (TickType_t)0) == pdTRUE)
            {
                // Check if engine is on
                if (info.flags & FLAG_ENGINE)
                {
                    if (!engine_on)
                    {
                        for (int i = 0; i < ENGINE_MAX_COUNT; ++i)
                        {
                            // Free semaphore
                            xSemaphoreGive(semaphore_engine);
                        }
                        LOG("Engines on\n")
                        engine_on = true;
                    }
                }
                else if (engine_on)
                {
                    // Take semaphore
                    for (int i = 0; i < ENGINE_MAX_COUNT; ++i)
                    {
                        // Free semaphore
                        xSemaphoreTake(semaphore_engine, portMAX_DELAY);
                    }
                    LOG("Engines off\n")
                    // Shutdown the whole system
                    {
                        digitalWrite(PIN_AEB_BUZZ, LOW);
                        digitalWrite(PIN_HEADLIGHTS, LOW);
                        digitalWrite(PIN_CAR_HORN, LOW);
                        analogWrite(PIN_MOTORS_L_EN, 0);
                        analogWrite(PIN_MOTORS_R_EN, 0);
                    }
                    engine_on = false;
                }
                xSemaphoreGive(semaphore_lock);
            }
            else
            {
                if (engine_on)
                {
                    // Take semaphore
                    for (int i = 0; i < ENGINE_MAX_COUNT; ++i)
                    {
                        // Free semaphore
                        xSemaphoreTake(semaphore_engine, portMAX_DELAY);
                    }
                    LOG("Engines off\n")
                    // Shutdown the whole system
                    {
                        digitalWrite(PIN_AEB_BUZZ, LOW);
                        digitalWrite(PIN_HEADLIGHTS, LOW);
                        digitalWrite(PIN_CAR_HORN, LOW);
                        analogWrite(PIN_MOTORS_L_EN, 0);
                        analogWrite(PIN_MOTORS_R_EN, 0);
                    }
                    engine_on = false;
                }
            }
            // Reset buffer position
            buff_ptr = (byte *)&info;
        }
    }
}

void loop()
{}
