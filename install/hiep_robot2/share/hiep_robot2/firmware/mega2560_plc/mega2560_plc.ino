#include <Arduino.h>

// ============================================================================
// USER HARDWARE CONFIG
// ----------------------------------------------------------------------------
// These are Arduino Mega pin numbers used only as a clean, compilable default.
// Change them to match the terminal/pin mapping of YOUR Mega2560 PLC board.
// Do not connect 24 V industrial sensors directly to bare Arduino MCU pins.
// Use the PLC board's conditioned/isolated inputs as intended by its hardware.
// ============================================================================

constexpr uint8_t CONVEYOR_COUNT = 4;
constexpr uint8_t SENSOR_COUNT = 8;

// Four conveyors. Each driver is assumed to have FORWARD + REVERSE + PWM.
// If your driver has a different interface, only the functions in the
// "Conveyor outputs" section need to be adapted.
const uint8_t CONV_PWM[CONVEYOR_COUNT] = {2, 3, 4, 5};
const uint8_t CONV_FWD[CONVEYOR_COUNT] = {22, 24, 26, 28};
const uint8_t CONV_REV[CONVEYOR_COUNT] = {23, 25, 27, 29};

// Two sensors per conveyor:
// Conveyor 1 -> A=30, B=31
// Conveyor 2 -> A=32, B=33
// Conveyor 3 -> A=34, B=35
// Conveyor 4 -> A=36, B=37
const uint8_t SENSOR_A[CONVEYOR_COUNT] = {30, 32, 34, 36};
const uint8_t SENSOR_B[CONVEYOR_COUNT] = {31, 33, 35, 37};

// Robot I/O.
constexpr uint8_t PIN_BUMPER_LEFT = 38;
constexpr uint8_t PIN_BUMPER_RIGHT = 39;
constexpr uint8_t PIN_EMG = 40;
constexpr uint8_t PIN_START = 41;
constexpr uint8_t PIN_STOP = 42;

// Four signal outputs: two on each side.
constexpr uint8_t PIN_LIGHT_LEFT_1 = 43;
constexpr uint8_t PIN_LIGHT_LEFT_2 = 44;
constexpr uint8_t PIN_LIGHT_RIGHT_1 = 45;
constexpr uint8_t PIN_LIGHT_RIGHT_2 = 46;
const uint8_t LIGHT_PINS[4] = {
  PIN_LIGHT_LEFT_1,
  PIN_LIGHT_LEFT_2,
  PIN_LIGHT_RIGHT_1,
  PIN_LIGHT_RIGHT_2
};

// Change these to suit the electrical behavior of the actual PLC board.
constexpr bool SENSOR_ACTIVE_HIGH = true;
constexpr bool BUMPER_ACTIVE_HIGH = true;
constexpr bool EMG_ACTIVE_HIGH = true;
constexpr bool START_ACTIVE_HIGH = true;
constexpr bool STOP_ACTIVE_HIGH = true;
constexpr bool OUTPUT_ACTIVE_HIGH = true;

// Input pin modes: INPUT or INPUT_PULLUP.
// Floating inputs read random values. If a switch/sensor contact goes straight
// to a bare Mega pin, use INPUT_PULLUP. For EMG/STOP wire a NORMALLY-CLOSED
// contact to GND with INPUT_PULLUP and *_ACTIVE_HIGH = true: a broken wire then
// reads as "pressed" (fail-safe). Keep INPUT if the PLC board's conditioned /
// isolated inputs already drive the pin actively.
constexpr uint8_t SENSOR_PIN_MODE = INPUT;
constexpr uint8_t SAFETY_PIN_MODE = INPUT;   // bumpers, EMG, START, STOP

// Receive means conveyor direction A -> B.
// Send means direction B -> A.
// Change a sign to -1 if that conveyor's physical motor direction is opposite.
const int8_t CONVEYOR_DIRECTION_SIGN[CONVEYOR_COUNT] = {1, 1, 1, 1};

// Local safety behavior.
constexpr bool BUMPER_STOPS_CONVEYORS = true;
constexpr uint32_t MAX_RUN_TIME_MS = 15000;  // Hard local maximum, even if HMI asks for infinity.
constexpr uint32_t STATUS_PERIOD_MS = 100;   // 10 Hz status to ROS.
constexpr uint32_t ID_PERIOD_MS = 1000;
constexpr uint32_t SERIAL_BAUD = 115200;

// ============================================================================
// Runtime state
// ============================================================================

enum ConveyorMode : uint8_t {
  MODE_IDLE = 0,
  MODE_RECEIVE = 1,
  MODE_SEND = 2,
  MODE_FAULT = 3,
};

struct ConveyorRuntime {
  ConveyorMode mode;
  uint8_t speedPercent;
  uint32_t startedMs;
  uint32_t requestedDurationMs;
  bool cargoPresent;
  bool sendSawSensorA;
  bool fault;
};

ConveyorRuntime conveyor[CONVEYOR_COUNT];
uint32_t statusSequence = 0;
uint8_t lightMask = 0;

// ============================================================================
// Helpers
// ============================================================================

bool readActive(uint8_t pin, bool activeHigh) {
  const bool raw = digitalRead(pin) == HIGH;
  return activeHigh ? raw : !raw;
}

void writeActive(uint8_t pin, bool on) {
  digitalWrite(pin, (on == OUTPUT_ACTIVE_HIGH) ? HIGH : LOW);
}

uint8_t speedPercentToPwm(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255U) / 100U);
}

bool sensorA(uint8_t index) {
  return readActive(SENSOR_A[index], SENSOR_ACTIVE_HIGH);
}

bool sensorB(uint8_t index) {
  return readActive(SENSOR_B[index], SENSOR_ACTIVE_HIGH);
}

bool bumperLeft() {
  return readActive(PIN_BUMPER_LEFT, BUMPER_ACTIVE_HIGH);
}

bool bumperRight() {
  return readActive(PIN_BUMPER_RIGHT, BUMPER_ACTIVE_HIGH);
}

bool emergencyActive() {
  return readActive(PIN_EMG, EMG_ACTIVE_HIGH);
}

bool startPressed() {
  return readActive(PIN_START, START_ACTIVE_HIGH);
}

bool stopPressed() {
  return readActive(PIN_STOP, STOP_ACTIVE_HIGH);
}

// ============================================================================
// Conveyor outputs
// ============================================================================

void stopConveyorHardware(uint8_t index) {
  writeActive(CONV_FWD[index], false);
  writeActive(CONV_REV[index], false);
  analogWrite(CONV_PWM[index], 0);
}

void driveConveyorHardware(uint8_t index, int direction, uint8_t speedPercent) {
  if (CONVEYOR_DIRECTION_SIGN[index] < 0) {
    direction = -direction;
  }

  // Always remove direction drive before changing direction.
  writeActive(CONV_FWD[index], false);
  writeActive(CONV_REV[index], false);
  analogWrite(CONV_PWM[index], 0);

  if (direction > 0) {
    writeActive(CONV_FWD[index], true);
  } else if (direction < 0) {
    writeActive(CONV_REV[index], true);
  } else {
    return;
  }

  analogWrite(CONV_PWM[index], speedPercentToPwm(speedPercent));
}

void setIdle(uint8_t index) {
  stopConveyorHardware(index);
  conveyor[index].mode = MODE_IDLE;
  conveyor[index].speedPercent = 0;
  conveyor[index].startedMs = 0;
  conveyor[index].requestedDurationMs = 0;
  conveyor[index].sendSawSensorA = false;
}

void stopAllConveyors() {
  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    setIdle(i);
  }
}

void startConveyor(uint8_t index, ConveyorMode mode, uint8_t speedPercent, uint32_t durationMs) {
  if (index >= CONVEYOR_COUNT) return;

  // Safety inputs have priority over software commands.
  if (emergencyActive() || stopPressed() ||
      (BUMPER_STOPS_CONVEYORS && (bumperLeft() || bumperRight()))) {
    setIdle(index);
    return;
  }

  conveyor[index].fault = false;
  conveyor[index].mode = mode;
  conveyor[index].speedPercent = constrain(speedPercent, 0, 100);
  conveyor[index].startedMs = millis();
  conveyor[index].requestedDurationMs = durationMs;
  conveyor[index].sendSawSensorA = false;

  if (conveyor[index].speedPercent == 0) {
    setIdle(index);
    return;
  }

  if (mode == MODE_RECEIVE) {
    // Sensor A = receiving end, sensor B = opposite end.
    // If cargo is already at B, do not drive into it.
    if (sensorB(index)) {
      conveyor[index].cargoPresent = true;
      setIdle(index);
      return;
    }
    driveConveyorHardware(index, +1, conveyor[index].speedPercent);
  } else if (mode == MODE_SEND) {
    driveConveyorHardware(index, -1, conveyor[index].speedPercent);
  } else {
    setIdle(index);
  }
}

// ============================================================================
// Signal lights
// ============================================================================

void applyLightMask(uint8_t mask) {
  lightMask = mask & 0x0F;
  for (uint8_t i = 0; i < 4; ++i) {
    writeActive(LIGHT_PINS[i], (lightMask & (1U << i)) != 0);
  }
}

// ============================================================================
// Local state machine
// ============================================================================

void updateConveyors() {
  const uint32_t now = millis();
  const bool safetyStop = emergencyActive() || stopPressed() ||
      (BUMPER_STOPS_CONVEYORS && (bumperLeft() || bumperRight()));

  if (safetyStop) {
    stopAllConveyors();
    return;
  }

  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    ConveyorRuntime &c = conveyor[i];

    // If either end sensor currently sees cargo while idle, remember that cargo
    // exists. We intentionally do NOT clear cargo just because both end sensors
    // are off: cargo can be physically between the two sensors.
    if (c.mode == MODE_IDLE && (sensorA(i) || sensorB(i))) {
      c.cargoPresent = true;
    }

    if (c.mode == MODE_IDLE || c.mode == MODE_FAULT) {
      continue;
    }

    const uint32_t elapsed = now - c.startedMs;

    // Requested HMI duration is a normal stop, not a fault. Duration 0 means
    // "run until sensor completes the transfer", but MAX_RUN_TIME_MS remains
    // an independent hard safety limit.
    if (c.requestedDurationMs > 0 && elapsed >= c.requestedDurationMs) {
      setIdle(i);
      continue;
    }

    if (elapsed >= MAX_RUN_TIME_MS) {
      stopConveyorHardware(i);
      c.mode = MODE_FAULT;
      c.fault = true;
      c.speedPercent = 0;
      continue;
    }

    if (c.mode == MODE_RECEIVE) {
      // Receive A -> B. Stop when cargo reaches sensor B.
      if (sensorB(i)) {
        c.cargoPresent = true;
        setIdle(i);
      }
    } else if (c.mode == MODE_SEND) {
      // Send B -> A. Observe cargo arriving at A, then stop after it clears A;
      // this means the cargo has left the conveyor through the A side.
      if (sensorA(i)) {
        c.sendSawSensorA = true;
      }
      if (c.sendSawSensorA && !sensorA(i)) {
        c.cargoPresent = false;
        setIdle(i);
      }
    }
  }
}

// ============================================================================
// Serial protocol
// Host -> PLC:
//   CV,<seq>,<id>,R|S|X,<speed>,<duration_ms>
//   LED,<seq>,<mask>
//   STOPALL,<seq>
// ============================================================================

void handleLine(char *line) {
  long seq = 0;
  int conveyorId = 0;
  char mode = 0;
  int speed = 0;
  unsigned long durationMs = 0;

  if (sscanf(line, "CV,%ld,%d,%c,%d,%lu", &seq, &conveyorId, &mode, &speed, &durationMs) == 5) {
    (void)seq;
    if (conveyorId < 1 || conveyorId > CONVEYOR_COUNT) return;
    const uint8_t index = static_cast<uint8_t>(conveyorId - 1);

    if (mode == 'R') {
      startConveyor(index, MODE_RECEIVE, static_cast<uint8_t>(constrain(speed, 0, 100)), durationMs);
    } else if (mode == 'S') {
      startConveyor(index, MODE_SEND, static_cast<uint8_t>(constrain(speed, 0, 100)), durationMs);
    } else if (mode == 'X') {
      conveyor[index].fault = false;
      setIdle(index);
    }
    return;
  }

  int ledMask = 0;
  if (sscanf(line, "LED,%ld,%d", &seq, &ledMask) == 2) {
    (void)seq;
    applyLightMask(static_cast<uint8_t>(ledMask));
    return;
  }

  if (sscanf(line, "STOPALL,%ld", &seq) == 1) {
    (void)seq;
    stopAllConveyors();
    return;
  }
}

// Non-blocking line assembler (replaces readBytesUntil, which returned partial
// lines on a >2 ms gap between bytes and blocked the safety loop).
void parseSerialInput() {
  static char buffer[128];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (len > 0 && !overflow) {
        buffer[len] = '\0';
        handleLine(buffer);
      }
      len = 0;
      overflow = false;
      continue;
    }

    if (len < sizeof(buffer) - 1) {
      buffer[len++] = static_cast<char>(c);
    } else {
      overflow = true;
    }
  }
}

// ============================================================================
// PLC -> ROS status
// ============================================================================

uint8_t buildSensorMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    if (sensorA(i)) mask |= 1U << (i * 2);
    if (sensorB(i)) mask |= 1U << (i * 2 + 1);
  }
  return mask;
}

uint8_t buildCargoMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    if (conveyor[i].cargoPresent) mask |= 1U << i;
  }
  return mask;
}

uint8_t buildRunningMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    if (conveyor[i].mode == MODE_RECEIVE || conveyor[i].mode == MODE_SEND) {
      mask |= 1U << i;
    }
  }
  return mask;
}

uint8_t buildFaultMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    if (conveyor[i].fault || conveyor[i].mode == MODE_FAULT) mask |= 1U << i;
  }
  return mask;
}

void publishStatus() {
  uint8_t bumperMask = 0;
  if (bumperLeft()) bumperMask |= 0x01;
  if (bumperRight()) bumperMask |= 0x02;

  ++statusSequence;
  Serial.print("PSTAT,");
  Serial.print(statusSequence);
  Serial.print(',');
  Serial.print(millis());
  Serial.print(',');
  Serial.print(buildSensorMask());
  Serial.print(',');
  Serial.print(bumperMask);
  Serial.print(',');
  Serial.print(emergencyActive() ? 1 : 0);
  Serial.print(',');
  Serial.print(startPressed() ? 1 : 0);
  Serial.print(',');
  Serial.print(stopPressed() ? 1 : 0);
  Serial.print(',');
  Serial.print(buildCargoMask());
  Serial.print(',');
  Serial.print(buildRunningMask());
  Serial.print(',');
  Serial.print(buildFaultMask());
  Serial.print(',');
  Serial.print(lightMask);

  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(static_cast<uint8_t>(conveyor[i].mode));
  }
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(2);

  for (uint8_t i = 0; i < CONVEYOR_COUNT; ++i) {
    pinMode(CONV_FWD[i], OUTPUT);
    pinMode(CONV_REV[i], OUTPUT);
    pinMode(CONV_PWM[i], OUTPUT);
    pinMode(SENSOR_A[i], SENSOR_PIN_MODE);
    pinMode(SENSOR_B[i], SENSOR_PIN_MODE);

    conveyor[i].mode = MODE_IDLE;
    conveyor[i].speedPercent = 0;
    conveyor[i].startedMs = 0;
    conveyor[i].requestedDurationMs = 0;
    conveyor[i].cargoPresent = false;
    conveyor[i].sendSawSensorA = false;
    conveyor[i].fault = false;
    stopConveyorHardware(i);
  }

  pinMode(PIN_BUMPER_LEFT, SAFETY_PIN_MODE);
  pinMode(PIN_BUMPER_RIGHT, SAFETY_PIN_MODE);
  pinMode(PIN_EMG, SAFETY_PIN_MODE);
  pinMode(PIN_START, SAFETY_PIN_MODE);
  pinMode(PIN_STOP, SAFETY_PIN_MODE);

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(LIGHT_PINS[i], OUTPUT);
  }
  applyLightMask(0);

  delay(50);
  Serial.println("ID,AGV_PLC_MEGA2560,0.4.0");
}

void loop() {
  static uint32_t lastStatusMs = 0;
  static uint32_t lastIdMs = 0;

  parseSerialInput();
  updateConveyors();

  const uint32_t now = millis();
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    publishStatus();
  }

  // Periodic identity makes passive auto-detection reliable even when the ROS
  // bridge starts long after the PLC firmware has already booted.
  if (now - lastIdMs >= ID_PERIOD_MS) {
    lastIdMs = now;
    Serial.println("ID,AGV_PLC_MEGA2560,0.4.0");
  }
}
