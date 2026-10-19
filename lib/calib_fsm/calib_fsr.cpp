#include "calib_fsr.h"
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr size_t kSensorCount = 8;
constexpr size_t kSampleCount = 100;
constexpr size_t kAverageHistoryDepth = 10;
constexpr uint32_t kSampleDelayMs = 5;
constexpr uint32_t kIdleBlinkMs = 800;
constexpr uint32_t kReadyBlinkMs = 250;
constexpr uint32_t kAutoTriggerDelayMs = 3000;  // WAIT_TIME requirement

const int sensorPins[kSensorCount] = {
    PIN_1, PIN_2, PIN_3, PIN_4,
    PIN_5, PIN_6, PIN_7, PIN_8
};

int currentSensor = -1;
bool awaitingSensorSelection = true;
uint32_t lastMenuPrint = 0;

enum class LedMode { Idle, Ready, Sampling };
LedMode ledMode = LedMode::Idle;
bool ledIsOn = false;
uint32_t lastBlinkMs = 0;
bool autoTriggerEnabled = false;
bool autoTriggerCounting = false;
bool autoTriggerAwaitingRelease = false;
bool autoTriggerNoticeShown = false;
uint32_t autoTriggerStartMs = 0;

bool sensorEnabled(int index);

struct AverageHistory {
    float values[kAverageHistoryDepth] = {0.0f};
    size_t count = 0;
    size_t nextIndex = 0;
};

AverageHistory averageHistories[kSensorCount];

void setLedOutput(bool on) {
    ledIsOn = on;
    digitalWrite(PIN_LED, on ? HIGH : LOW);
}

void setLedMode(LedMode mode) {
    ledMode = mode;
    lastBlinkMs = millis();
    if (mode == LedMode::Sampling) {
        setLedOutput(true);
    } else {
        setLedOutput(false);
    }
}

void armAutoTrigger() {
    autoTriggerEnabled = true;
    autoTriggerCounting = false;
    autoTriggerAwaitingRelease = false;
    autoTriggerNoticeShown = false;
    autoTriggerStartMs = 0;
}

void disarmAutoTrigger() {
    autoTriggerEnabled = false;
    autoTriggerCounting = false;
    autoTriggerAwaitingRelease = false;
    autoTriggerNoticeShown = false;
    autoTriggerStartMs = 0;
}

void resetSensorHistory(int sensorIndex) {
    if (!sensorEnabled(sensorIndex)) {
        return;
    }
    AverageHistory &history = averageHistories[sensorIndex];
    history.count = 0;
    history.nextIndex = 0;
    for (float &value : history.values) {
        value = 0.0f;
    }
}

void recordAverage(int sensorIndex, float average) {
    if (!sensorEnabled(sensorIndex)) {
        return;
    }
    AverageHistory &history = averageHistories[sensorIndex];
    history.values[history.nextIndex] = average;
    history.nextIndex = (history.nextIndex + 1) % kAverageHistoryDepth;
    if (history.count < kAverageHistoryDepth) {
        ++history.count;
    }
}

void printAverageHistory(int sensorIndex) {
    if (!sensorEnabled(sensorIndex)) {
        Serial.println(F("Selected sensor is disabled."));
        return;
    }
    const AverageHistory &history = averageHistories[sensorIndex];
    if (history.count == 0) {
        Serial.printf("[Sensor %u] History empty. Run sampling first.\n",
                      static_cast<unsigned>(sensorIndex + 1));
        return;
    }

    Serial.printf("[Sensor %u] Last %u average readings:\n",
                  static_cast<unsigned>(sensorIndex + 1),
                  static_cast<unsigned>(history.count));

    const size_t startIndex = (history.nextIndex + kAverageHistoryDepth - history.count) % kAverageHistoryDepth;
    float sum = 0.0f;
    Serial.print("  Values: [");
    for (size_t i = 0; i < history.count; ++i) {
        const size_t idx = (startIndex + i) % kAverageHistoryDepth;
        const float value = history.values[idx];
        sum += value;
        Serial.printf("%.2f", value);
        if (i + 1 < history.count) {
            Serial.print(", ");
        }
    }
    Serial.println("]");

    Serial.printf("  Sum: %.2f\n", sum);
    Serial.printf("  Average of history: %.2f\n", sum / static_cast<float>(history.count));
}

void handlePostSample() {
    if (!autoTriggerEnabled) {
        return;
    }
    autoTriggerCounting = false;
    autoTriggerStartMs = 0;
    autoTriggerNoticeShown = false;
    autoTriggerAwaitingRelease = true;
}

void updateLedBlink() {
    if (ledMode == LedMode::Sampling) {
        return;
    }

    const uint32_t interval = (ledMode == LedMode::Idle) ? kIdleBlinkMs : kReadyBlinkMs;
    if (millis() - lastBlinkMs >= interval) {
        setLedOutput(!ledIsOn);
        lastBlinkMs = millis();
    }
}

bool isLineEnding(char c) {
    return c == '\n' || c == '\r';
}

bool sensorEnabled(int index) {
    return index >= 0 && index < static_cast<int>(kSensorCount) && sensorPins[index] >= 0;
}

void printSensorMenu() {
    Serial.println();
    Serial.println("=== FSR Calibration Tool ===");
    Serial.println(F("Select a sensor to calibrate (1-8):"));
    for (size_t i = 0; i < kSensorCount; ++i) {
        if (sensorPins[i] < 0) {
            Serial.printf("  %u) Sensor disabled\n", static_cast<unsigned>(i + 1));
        } else {
            Serial.printf("  %u) Pin %d\n", static_cast<unsigned>(i + 1), sensorPins[i]);
        }
    }
    Serial.println(F("Enter the sensor number via Serial."));
}

void printSensorCommands() {
    Serial.println();
    Serial.printf("[Sensor %u] Commands:\n", static_cast<unsigned>(currentSensor + 1));
    Serial.println(F("  s - start sampling (100 reads)"));
    Serial.println(F("  p - print last 10 averages"));
    Serial.println(F("  e - exit to sensor selection"));
}

void printAwaitingCommand() {
    Serial.println(F("Waiting for command ('s' to sample, 'e' to exit, 'p' to print 10 last values)..."));
}

void sampleSensor(int sensorIndex) {
    if (!sensorEnabled(sensorIndex)) {
        Serial.println(F("Selected sensor is disabled."));
        return;
    }

    setLedMode(LedMode::Sampling);

    uint64_t sum = 0;
    Serial.printf("\n[Sensor %u] Sampling %u readings on pin %d...\n",
                  static_cast<unsigned>(sensorIndex + 1),
                  static_cast<unsigned>(kSampleCount),
                  sensorPins[sensorIndex]);

    for (size_t i = 0; i < kSampleCount; ++i) {
        uint16_t value = analogRead((uint8_t)sensorPins[sensorIndex]);
        sum += value;
        // Serial.printf("  #%03u -> %u\n", static_cast<unsigned>(i + 1), value);
        delay(kSampleDelayMs);
    }

    const float average = static_cast<float>(sum) / static_cast<float>(kSampleCount);
    recordAverage(sensorIndex, average);
    Serial.printf("[Sensor %u] Average: %.2f\n", static_cast<unsigned>(sensorIndex + 1), average);
    setLedMode(LedMode::Ready);
    printAwaitingCommand();
    handlePostSample();
}

void handleSensorSelection(char input) {
    if (!isdigit(static_cast<unsigned char>(input))) {
        Serial.println(F("Please enter a digit between 1 and 8."));
        return;
    }

    const int sensorIndex = input - '1';
    if (!sensorEnabled(sensorIndex)) {
        Serial.printf("Sensor %d is disabled or unavailable.\n", sensorIndex + 1);
        return;
    }

    currentSensor = sensorIndex;
    awaitingSensorSelection = false;
    resetSensorHistory(currentSensor);
    setLedMode(LedMode::Ready);
    Serial.printf("Selected sensor %d (pin %d).\n", sensorIndex + 1, sensorPins[sensorIndex]);
    printSensorCommands();
    printAwaitingCommand();
    armAutoTrigger();
}

void handleSensorCommand(char input) {
    switch (tolower(static_cast<unsigned char>(input))) {
        case 's':
            sampleSensor(currentSensor);
            break;
        case 'p':
            printAverageHistory(currentSensor);
            printAwaitingCommand();
            break;
        case 'e':
            awaitingSensorSelection = true;
            currentSensor = -1;
            setLedMode(LedMode::Idle);
            printSensorMenu();
            disarmAutoTrigger();
            break;
        default:
            Serial.println(F("Unknown command. Use 's' to sample or 'e' to exit."));
            printAwaitingCommand();
            break;
    }
}

void processAutoTrigger() {
    if (!autoTriggerEnabled || awaitingSensorSelection || currentSensor < 0 ||
        ledMode == LedMode::Sampling || !sensorEnabled(currentSensor)) {
        return;
    }

    const uint16_t value = analogRead((uint8_t)sensorPins[currentSensor]);
    if (value == 0) {
        if (autoTriggerAwaitingRelease) {
            autoTriggerAwaitingRelease = false;
            Serial.printf("[Sensor %u] Sensor released. Auto trigger ready.\n",
                          static_cast<unsigned>(currentSensor + 1));
        }
        autoTriggerCounting = false;
        autoTriggerStartMs = 0;
        autoTriggerNoticeShown = false;
        return;
    }

    if (autoTriggerAwaitingRelease) {
        return;
    }

    if (!autoTriggerCounting) {
        autoTriggerCounting = true;
        autoTriggerStartMs = millis();
        if (!autoTriggerNoticeShown) {
            autoTriggerNoticeShown = true;
            Serial.printf("[Sensor %u] Activity detected. Auto-sampling in %lu ms.\n",
                          static_cast<unsigned>(currentSensor + 1),
                          static_cast<unsigned long>(kAutoTriggerDelayMs));
        }
        return;
    }

    if (millis() - autoTriggerStartMs >= kAutoTriggerDelayMs) {
        autoTriggerCounting = false;
        autoTriggerStartMs = 0;
        autoTriggerNoticeShown = false;
        Serial.println(F("Auto trigger reached. Starting sampling..."));
        sampleSensor(currentSensor);
    }
}
}  // namespace

void calib_fsr_setup() {
    pinMode(PIN_LED, OUTPUT);
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);
    for (size_t i = 0; i < kSensorCount; ++i) {
        if (sensorPins[i] >= 0) {
            pinMode(sensorPins[i], INPUT);
        }
        resetSensorHistory(static_cast<int>(i));
    }
    setLedMode(LedMode::Idle);
    disarmAutoTrigger();
    lastMenuPrint = millis();
    printSensorMenu();
}

void calib_fsr_loop(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        updateLedBlink();
        processAutoTrigger();

        if (awaitingSensorSelection && millis() - lastMenuPrint > 4000) {
            printSensorMenu();
            lastMenuPrint = millis();
        }

        if (!Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const char input = Serial.read();
        if (isLineEnding(input)) {
            continue;
        }

        if (awaitingSensorSelection) {
            handleSensorSelection(input);
            lastMenuPrint = millis();
        } else {
            handleSensorCommand(input);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
