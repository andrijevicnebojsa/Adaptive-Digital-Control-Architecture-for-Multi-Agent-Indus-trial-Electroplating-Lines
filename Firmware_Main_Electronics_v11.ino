#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

/*
  Firmware_Main_Electronics_FINAL.ino
  Final supplementary firmware aligned with the manuscript:
  - Dual trolley asynchronous FSM
  - Discrete TPS-based positioning
  - Virtual Process Groups (zinc, passivation)
  - Temporal FIFO retrieval by readiness time
  - Layered safety and emergency stop
  - Non-blocking thermal regulation

  Notes:
  1) This sketch is a clean, publication-oriented reference implementation.
  2) It is designed to reflect the architecture and I/O mapping described in the paper.
  3) Field commissioning may still require hardware-specific timing tuning.
*/

// -----------------------------------------------------------------------------
//  GLOBAL CONSTANTS
// -----------------------------------------------------------------------------
constexpr uint8_t TANK_COUNT = 18;
constexpr uint8_t TEMP_ZONE_COUNT = 7;
constexpr uint8_t TASK_LIMIT = 50;
constexpr uint8_t END_MARKER = 255;

constexpr uint8_t ONE_WIRE_PIN = 11;
constexpr uint8_t EMG_STOP_PIN = 18;
constexpr uint8_t ENC_A_PIN = 19;      // single-channel pulse input for local menu stepping
constexpr uint8_t ENC_SW_PIN = 12;     // encoder push button
constexpr uint8_t LCD_I2C_ADDR = 0x27;

constexpr uint8_t GROUP_ZINC = 20;         // logical group code in upper 5 bits
constexpr uint8_t GROUP_PASSIVATION = 21;  // logical group code in upper 5 bits

constexpr uint16_t EEPROM_TASKS_T1 = 200;
constexpr uint16_t EEPROM_TASKS_T2 = 300;
constexpr uint16_t EEPROM_WAIT_BASE = 500; // station-indexed dwell times in seconds

constexpr uint8_t SAFE_GAP_TANKS = 2;
constexpr unsigned long TRAVEL_TIME_PER_TANK_MS = 2000UL;
constexpr unsigned long DRUM_ACTUATION_MS = 1500UL;
constexpr unsigned long DT_VERIFY_TIMEOUT_MS = 1200UL;  // Max wait for DT after reaching DL
constexpr bool DT_ACTIVE_LOW = true;                   // Set according to wiring (true: LOW=active)
constexpr unsigned long HMI_REFRESH_MS = 250UL;
constexpr unsigned long TEMP_REQUEST_PERIOD_MS = 3000UL;
constexpr unsigned long TEMP_CONVERSION_DELAY_MS = 900UL;

constexpr float ZONE_SETPOINTS[TEMP_ZONE_COUNT] = {60.0f, 30.0f, 30.0f, 20.0f, 20.0f, 20.0f, 20.0f};
constexpr float TEMP_HYST = 1.0f;
constexpr unsigned long PREDICTIVE_TRAVEL_PER_TANK_MS = 2000UL;
constexpr unsigned long PREDICTIVE_LOAD_UNLOAD_MS = 3000UL;
constexpr unsigned long PREDICTIVE_MARGIN_MS = 1000UL;
constexpr unsigned long PREDICTIVE_MAX_CYCLE_MS = 45UL * 60UL * 1000UL;

// -----------------------------------------------------------------------------
//  PIN MAP STRUCTURES
// -----------------------------------------------------------------------------
struct TrolleyPins {
  uint8_t cmdLeft;
  uint8_t cmdRight;
  uint8_t cmdFast;
  uint8_t cmdDrumUp;
  uint8_t cmdDrumDown;
  uint8_t tpsStartPin;      // 5 consecutive TPS inputs
  uint8_t pinDU;
  uint8_t pinDL;
  uint8_t pinDT;            // Drum-in-tank confirmation (DT)
  uint8_t pinDUS;
  uint8_t pinDLS;
  uint8_t pinTCR;
  uint8_t pinTCL;
};

const TrolleyPins T1_PINS = {
  2, 3, 4, 8, 9,
  22,
  27, 28,
  17,            // DT (Trolley 1)
  29, 35,
  36, 37
};

const TrolleyPins T2_PINS = {
  5, 6, 7, 14, 15,
  30,
  38, 39,
  10,            // DT (Trolley 2)
  40, 41,
  42, 43
};

const uint8_t HEATER_PINS[TEMP_ZONE_COUNT] = {46, 47, 48, 49, 50, 51, 52};
const uint8_t FAN_PINS[4] = {53, 44, 45, 16};

// -----------------------------------------------------------------------------
//  HARDWARE OBJECTS
// -----------------------------------------------------------------------------
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensors(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

// -----------------------------------------------------------------------------
//  FSM DEFINITIONS
// -----------------------------------------------------------------------------
enum MainPhase : int8_t {
  PHASE_END = -1,
  PHASE_DISPATCH = 0,
  PHASE_PROCESS = 7,
  PHASE_MOVE_RIGHT = 8,
  PHASE_MOVE_LEFT = 9,
  PHASE_HOMING = 10,
  PHASE_WAIT_RESOURCE = 11,
  PHASE_RETRIEVE_GROUP = 12
};

enum DrumPhase : int8_t {
  DRUM_IDLE = 0,
  DRUM_LOWERING = 1,
  DRUM_WAIT_DT = 2,
  DRUM_HOLDING = 3,
  DRUM_RAISING = 4
};

struct TrolleyState {
  int8_t mainPhase = PHASE_HOMING;
  int8_t drumPhase = DRUM_IDLE;
  uint8_t currentTank = 0;
  uint8_t targetTank = 0;
  uint8_t logicalTarget = 0;
  uint8_t assignedPhysicalTank = 0;
  uint8_t taskPointer = 0;
  uint8_t processType = 0;
  bool ready = false;
  bool carryingDrum = true;
  unsigned long phaseStartMs = 0;
  unsigned long processStartMs = 0;
};

struct GroupInfo {
  const uint8_t* members;
  uint8_t count;
};

const uint8_t ZINC_GROUP_MEMBERS[] = {5, 6, 7, 8};
const uint8_t PASSIVATION_GROUP_MEMBERS[] = {12, 13};

const GroupInfo ZINC_GROUP = {ZINC_GROUP_MEMBERS, 4};
const GroupInfo PASSIVATION_GROUP = {PASSIVATION_GROUP_MEMBERS, 2};

struct PredictiveWindow {
  uint8_t startTank;
  uint8_t endTank;
  unsigned long enterMs;
  unsigned long leaveMs;
};

TrolleyState T1;
TrolleyState T2;

bool tankOccupied[TANK_COUNT + 1] = {false};
bool tankReady[TANK_COUNT + 1] = {false};
unsigned long tankReadyTime[TANK_COUNT + 1] = {0};

bool emergencyLatched = false;
bool systemRunning = false;

float zoneTemps[TEMP_ZONE_COUNT] = {0.0f};
bool tempRequestInFlight = false;
unsigned long tempRequestStartedMs = 0;
unsigned long lastTempRequestMs = 0;

long hmiPulseCounter = 0;
int menuIndex = 0;
bool lastEncAState = HIGH;
bool lastBtnState = HIGH;
unsigned long lastHmiRefreshMs = 0;

// -----------------------------------------------------------------------------
//  FORWARD DECLARATIONS
// -----------------------------------------------------------------------------
const TrolleyPins& pinsFor(uint8_t id);
TrolleyState& trolleyFor(uint8_t id);
TrolleyState& otherTrolleyFor(uint8_t id);
void stopTrolleyOutputs(uint8_t id);
uint8_t readTPS(uint8_t id);
void updateCurrentPosition(uint8_t id);
void executeTrolleyLogic(uint8_t id);
void loadNextTask(uint8_t id);
void resolveDispatch(uint8_t id);
void handleMovePhase(uint8_t id);
void handleProcessCycle(uint8_t id);
void handleRetrieveGroup(uint8_t id);
void releaseTank(uint8_t tank);
int getAvailablePhysicalTank(uint8_t logicalGroup);
int selectReadyTankByTemporalFIFO(uint8_t logicalGroup);
bool isSharedTrackConflict(uint8_t id, uint8_t candidateTarget);
bool validateTableIntegrity();
bool validateStartupRecipes();
bool isDrumLowerSafe(uint8_t id);
bool isDrumRaiseSafe(uint8_t id);
bool isMotionDirectionSafe(uint8_t id, bool moveRight);
unsigned long estimateTransitionDurationMs(uint8_t fromTank, uint8_t toTank, unsigned long dwellMs);
void serviceThermalControl();
void requestTemperaturesNonBlocking();
void readTemperaturesIfReady();
void serviceHMI();
void refreshLCD();
void handleEmergencyStop();
void emergencyStopISR();

uint8_t decodeLogicalTarget(uint8_t raw);
uint8_t decodeProcessType(uint8_t raw);
unsigned long getTaskDwellMs(uint8_t trolleyId, uint8_t taskPointer);
uint8_t getHomePosition(uint8_t trolleyId);
const GroupInfo* groupForLogicalId(uint8_t logicalGroup);

// -----------------------------------------------------------------------------
//  HELPERS
// -----------------------------------------------------------------------------
const TrolleyPins& pinsFor(uint8_t id) {
  return (id == 0) ? T1_PINS : T2_PINS;
}

TrolleyState& trolleyFor(uint8_t id) {
  return (id == 0) ? T1 : T2;
}

TrolleyState& otherTrolleyFor(uint8_t id) {
  return (id == 0) ? T2 : T1;
}

uint8_t decodeLogicalTarget(uint8_t raw) {
  return static_cast<uint8_t>(raw >> 3);
}

uint8_t decodeProcessType(uint8_t raw) {
  return static_cast<uint8_t>(raw & 0x07);
}

unsigned long getTaskDwellMs(uint8_t trolleyId, uint8_t taskPointer) {
  const uint16_t base = (trolleyId == 0) ? EEPROM_TASKS_T1 : EEPROM_TASKS_T2;
  const uint8_t raw = EEPROM.read(base + taskPointer);
  const uint8_t logical = decodeLogicalTarget(raw);
  return static_cast<unsigned long>(EEPROM.read(EEPROM_WAIT_BASE + logical)) * 1000UL;
}

uint8_t getHomePosition(uint8_t trolleyId) {
  const uint16_t base = (trolleyId == 0) ? EEPROM_TASKS_T1 : EEPROM_TASKS_T2;
  const uint8_t raw = EEPROM.read(base);
  return decodeLogicalTarget(raw);
}

const GroupInfo* groupForLogicalId(uint8_t logicalGroup) {
  if (logicalGroup == GROUP_ZINC) return &ZINC_GROUP;
  if (logicalGroup == GROUP_PASSIVATION) return &PASSIVATION_GROUP;
  return nullptr;
}

bool isDrumLowerSafe(uint8_t id) {
  const TrolleyPins& p = pinsFor(id);
  return (digitalRead(p.pinDLS) == HIGH) && (digitalRead(p.pinDL) == HIGH || digitalRead(p.pinDT) == HIGH);
}

bool isDrumRaiseSafe(uint8_t id) {
  const TrolleyPins& p = pinsFor(id);
  return (digitalRead(p.pinDUS) == HIGH) && (digitalRead(p.pinDU) == HIGH || digitalRead(p.pinDT) == LOW);
}

bool isMotionDirectionSafe(uint8_t id, bool moveRight) {
  const TrolleyPins& p = pinsFor(id);
  if (moveRight) {
    return digitalRead(p.pinTCR) == HIGH;
  }
  return digitalRead(p.pinTCL) == HIGH;
}

static inline uint8_t u8min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }
static inline uint8_t u8max(uint8_t a, uint8_t b) { return (a > b) ? a : b; }

unsigned long estimateTransitionDurationMs(uint8_t fromTank, uint8_t toTank, unsigned long dwellMs) {
  const unsigned long travelMs = static_cast<unsigned long>(abs(static_cast<int>(toTank) - static_cast<int>(fromTank))) * PREDICTIVE_TRAVEL_PER_TANK_MS;
  return PREDICTIVE_LOAD_UNLOAD_MS + travelMs + dwellMs + PREDICTIVE_LOAD_UNLOAD_MS;
}

void emergencyStopISR() {
  emergencyLatched = true;
}

void stopTrolleyOutputs(uint8_t id) {
  const TrolleyPins& p = pinsFor(id);
  digitalWrite(p.cmdLeft, LOW);
  digitalWrite(p.cmdRight, LOW);
  digitalWrite(p.cmdFast, LOW);
  digitalWrite(p.cmdDrumUp, LOW);
  digitalWrite(p.cmdDrumDown, LOW);
}

bool isSharedTrackConflict(uint8_t id, uint8_t candidateTarget) {
  const TrolleyState& self = trolleyFor(id);
  const TrolleyState& other = otherTrolleyFor(id);

  // Conservative shared-track rule: prevent movement into candidate target if the
  // resulting separation is below SAFE_GAP_TANKS or if the trolleys already occupy
  // nearby positions during active movement.
  if (other.mainPhase == PHASE_END) return false;
  if (abs(static_cast<int>(candidateTarget) - static_cast<int>(other.currentTank)) < SAFE_GAP_TANKS) {
    return true;
  }
  if (abs(static_cast<int>(self.currentTank) - static_cast<int>(other.currentTank)) < SAFE_GAP_TANKS &&
      (self.mainPhase == PHASE_MOVE_LEFT || self.mainPhase == PHASE_MOVE_RIGHT ||
       other.mainPhase == PHASE_MOVE_LEFT || other.mainPhase == PHASE_MOVE_RIGHT)) {
    return true;
  }
  return false;
}

int getAvailablePhysicalTank(uint8_t logicalGroup) {
  const GroupInfo* group = groupForLogicalId(logicalGroup);
  if (group == nullptr) return -1;

  for (uint8_t i = 0; i < group->count; ++i) {
    const uint8_t tank = group->members[i];
    if (!tankOccupied[tank]) {
      tankOccupied[tank] = true;    // reserve immediately
      tankReady[tank] = false;
      tankReadyTime[tank] = 0;
      return tank;
    }
  }
  return -1;
}

int selectReadyTankByTemporalFIFO(uint8_t logicalGroup) {
  const GroupInfo* group = groupForLogicalId(logicalGroup);
  if (group == nullptr) return -1;

  int selected = -1;
  unsigned long oldestReady = 0UL;
  bool first = true;

  for (uint8_t i = 0; i < group->count; ++i) {
    const uint8_t tank = group->members[i];
    if (tankReady[tank]) {
      if (first || tankReadyTime[tank] < oldestReady) {
        oldestReady = tankReadyTime[tank];
        selected = tank;
        first = false;
      }
    }
  }
  return selected;
}

void releaseTank(uint8_t tank) {
  if (tank >= 1 && tank <= TANK_COUNT) {
    tankOccupied[tank] = false;
    tankReady[tank] = false;
    tankReadyTime[tank] = 0UL;
  }
}

// -----------------------------------------------------------------------------
//  SETUP / LOOP
// -----------------------------------------------------------------------------
void setup() {
  const uint8_t outputPins[] = {
    T1_PINS.cmdLeft, T1_PINS.cmdRight, T1_PINS.cmdFast, T1_PINS.cmdDrumUp, T1_PINS.cmdDrumDown,
    T2_PINS.cmdLeft, T2_PINS.cmdRight, T2_PINS.cmdFast, T2_PINS.cmdDrumUp, T2_PINS.cmdDrumDown,
    HEATER_PINS[0], HEATER_PINS[1], HEATER_PINS[2], HEATER_PINS[3], HEATER_PINS[4], HEATER_PINS[5], HEATER_PINS[6],
    FAN_PINS[0], FAN_PINS[1], FAN_PINS[2], FAN_PINS[3]
  };

  for (uint8_t i = 0; i < sizeof(outputPins); ++i) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }

  // All discrete field inputs, pull-up conditioned.
  for (uint8_t p = 22; p <= 43; ++p) {
    pinMode(p, INPUT_PULLUP);
  }
  // DT inputs are outside the 22–43 block in the final mapping.
  pinMode(T1_PINS.pinDT, INPUT_PULLUP);
  pinMode(T2_PINS.pinDT, INPUT_PULLUP);
  pinMode(EMG_STOP_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMG_STOP_PIN), emergencyStopISR, FALLING);

  lcd.init();
  lcd.backlight();
  tempSensors.begin();
  tempSensors.setWaitForConversion(false);

  T1.mainPhase = PHASE_HOMING;
  T2.mainPhase = PHASE_HOMING;
  T1.taskPointer = 0;
  T2.taskPointer = 0;

  if (!validateTableIntegrity() || !validateStartupRecipes()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Recipe/Table Error");
    while (true) {
      handleEmergencyStop();
      delay(100);
    }
  }
}

void loop() {
  handleEmergencyStop();
  if (emergencyLatched) {
    delay(20);
    return;
  }

  updateCurrentPosition(0);
  updateCurrentPosition(1);

  executeTrolleyLogic(0);
  executeTrolleyLogic(1);

  serviceThermalControl();
  serviceHMI();
}

// -----------------------------------------------------------------------------
//  VALIDATION OF STORED TABLES
// -----------------------------------------------------------------------------
bool validateTableIntegrity() {
  // Basic integrity checks: task lists must contain valid logical targets,
  // process types in 0..7, and must terminate with END_MARKER within TASK_LIMIT.
  for (uint8_t trolleyId = 0; trolleyId < 2; ++trolleyId) {
    const uint16_t base = (trolleyId == 0) ? EEPROM_TASKS_T1 : EEPROM_TASKS_T2;
    bool foundEnd = false;

    for (uint8_t i = 0; i < TASK_LIMIT; ++i) {
      const uint8_t raw = EEPROM.read(base + i);
      if (raw == END_MARKER) {
        foundEnd = true;
        break;
      }
      const uint8_t logical = decodeLogicalTarget(raw);
      const uint8_t proc = decodeProcessType(raw);
      if (proc > 7) return false;
      if (!(logical >= 1 && logical <= TANK_COUNT) && logical != GROUP_ZINC && logical != GROUP_PASSIVATION) {
        return false;
      }
    }
    if (!foundEnd) return false;
  }
  return true;
}

bool validateStartupRecipes() {
  const uint8_t home1 = getHomePosition(0);
  const uint8_t home2 = getHomePosition(1);
  if (home1 < 1 || home1 > TANK_COUNT || home2 < 1 || home2 > TANK_COUNT) return false;
  if (abs(static_cast<int>(home1) - static_cast<int>(home2)) < SAFE_GAP_TANKS) return false;

  PredictiveWindow windows[2][TASK_LIMIT] = {};
  uint8_t counts[2] = {0, 0};

  for (uint8_t trolleyId = 0; trolleyId < 2; ++trolleyId) {
    const uint16_t base = (trolleyId == 0) ? EEPROM_TASKS_T1 : EEPROM_TASKS_T2;
    uint8_t current = getHomePosition(trolleyId);
    unsigned long timelineMs = 0;

    for (uint8_t i = 0; i < TASK_LIMIT; ++i) {
      const uint8_t raw = EEPROM.read(base + i);
      if (raw == END_MARKER) break;

      const uint8_t logical = decodeLogicalTarget(raw);
      const GroupInfo* group = groupForLogicalId(logical);
      uint8_t predictedTarget = logical;
      if (group != nullptr) {
        predictedTarget = group->members[0];
      }
      if (predictedTarget < 1 || predictedTarget > TANK_COUNT) return false;

      const unsigned long dwellMs = getTaskDwellMs(trolleyId, i);
      if (dwellMs == 0 || dwellMs > PREDICTIVE_MAX_CYCLE_MS) return false;

      PredictiveWindow &w = windows[trolleyId][counts[trolleyId]++];
      w.startTank = current;
      w.endTank = predictedTarget;
      w.enterMs = timelineMs;
      timelineMs += estimateTransitionDurationMs(current, predictedTarget, dwellMs);
      w.leaveMs = timelineMs;
      current = predictedTarget;
    }
  }

  for (uint8_t i = 0; i < counts[0]; ++i) {
    for (uint8_t j = 0; j < counts[1]; ++j) {
      const bool overlap = !(windows[0][i].leaveMs + PREDICTIVE_MARGIN_MS < windows[1][j].enterMs ||
                             windows[1][j].leaveMs + PREDICTIVE_MARGIN_MS < windows[0][i].enterMs);
      if (!overlap) continue;

      const uint8_t minA = u8min(windows[0][i].startTank, windows[0][i].endTank);
      const uint8_t maxA = u8max(windows[0][i].startTank, windows[0][i].endTank);
      const uint8_t minB = u8min(windows[1][j].startTank, windows[1][j].endTank);
      const uint8_t maxB = u8max(windows[1][j].startTank, windows[1][j].endTank);

      if (!(maxA + SAFE_GAP_TANKS <= minB || maxB + SAFE_GAP_TANKS <= minA)) {
        return false;
      }
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
//  POSITION RECONSTRUCTION
// -----------------------------------------------------------------------------
uint8_t readTPS(uint8_t id) {
  const TrolleyPins& p = pinsFor(id);
  uint8_t station = 0;
  for (uint8_t i = 0; i < 5; ++i) {
    // Active-low field wiring.
    if (digitalRead(p.tpsStartPin + i) == LOW) {
      station |= (1U << i);
    }
  }
  return station;
}

void updateCurrentPosition(uint8_t id) {
  trolleyFor(id).currentTank = readTPS(id);
}

// -----------------------------------------------------------------------------
//  MAIN TROLLEY LOGIC
// -----------------------------------------------------------------------------
void executeTrolleyLogic(uint8_t id) {
  TrolleyState& t = trolleyFor(id);

  switch (t.mainPhase) {
    case PHASE_HOMING:
      {
        const uint8_t home = getHomePosition(id);
        t.targetTank = home;
        if (t.currentTank == home && t.currentTank > 0) {
          stopTrolleyOutputs(id);
          t.ready = true;
          if (T1.ready && T2.ready) {
            systemRunning = true;
            T1.mainPhase = PHASE_DISPATCH;
            T2.mainPhase = PHASE_DISPATCH;
          }
        } else if (t.currentTank > 0 && t.currentTank < home && !isSharedTrackConflict(id, home)) {
          t.mainPhase = PHASE_MOVE_RIGHT;
          t.phaseStartMs = millis();
        } else if (t.currentTank > home && !isSharedTrackConflict(id, home)) {
          t.mainPhase = PHASE_MOVE_LEFT;
          t.phaseStartMs = millis();
        }
      }
      break;

    case PHASE_DISPATCH:
      loadNextTask(id);
      resolveDispatch(id);
      break;

    case PHASE_WAIT_RESOURCE:
      resolveDispatch(id);
      break;

    case PHASE_MOVE_LEFT:
    case PHASE_MOVE_RIGHT:
      handleMovePhase(id);
      break;

    case PHASE_PROCESS:
      handleProcessCycle(id);
      break;

    case PHASE_RETRIEVE_GROUP:
      handleRetrieveGroup(id);
      break;

    case PHASE_END:
    default:
      stopTrolleyOutputs(id);
      break;
  }
}

void loadNextTask(uint8_t id) {
  TrolleyState& t = trolleyFor(id);
  const uint16_t base = (id == 0) ? EEPROM_TASKS_T1 : EEPROM_TASKS_T2;
  const uint8_t raw = EEPROM.read(base + t.taskPointer);

  if (raw == END_MARKER) {
    t.mainPhase = PHASE_END;
    stopTrolleyOutputs(id);
    return;
  }

  t.logicalTarget = decodeLogicalTarget(raw);
  t.processType = decodeProcessType(raw);
  t.assignedPhysicalTank = 0;
}

void resolveDispatch(uint8_t id) {
  TrolleyState& t = trolleyFor(id);
  if (t.mainPhase == PHASE_END) return;

  // Grouped retrieval commands: if the command points to a group and trolley is not
  // carrying a drum, interpret it as grouped retrieval by Temporal FIFO.
  if ((t.logicalTarget == GROUP_ZINC || t.logicalTarget == GROUP_PASSIVATION) && !t.carryingDrum) {
    const int readyTank = selectReadyTankByTemporalFIFO(t.logicalTarget);
    if (readyTank < 0) {
      t.mainPhase = PHASE_WAIT_RESOURCE;
      return;
    }
    t.assignedPhysicalTank = static_cast<uint8_t>(readyTank);
    t.targetTank = static_cast<uint8_t>(readyTank);
    t.mainPhase = PHASE_RETRIEVE_GROUP;
  }
  // Grouped processing commands for a trolley carrying a fresh drum.
  else if (t.logicalTarget == GROUP_ZINC || t.logicalTarget == GROUP_PASSIVATION) {
    if (t.assignedPhysicalTank == 0) {
      const int assigned = getAvailablePhysicalTank(t.logicalTarget);
      if (assigned < 0) {
        t.mainPhase = PHASE_WAIT_RESOURCE;
        return;
      }
      t.assignedPhysicalTank = static_cast<uint8_t>(assigned);
    }
    t.targetTank = t.assignedPhysicalTank;
    t.mainPhase = (t.currentTank < t.targetTank) ? PHASE_MOVE_RIGHT :
                  (t.currentTank > t.targetTank) ? PHASE_MOVE_LEFT : PHASE_PROCESS;
    t.phaseStartMs = millis();
  }
  // Fixed station commands.
  else {
    t.targetTank = t.logicalTarget;
    if (isSharedTrackConflict(id, t.targetTank)) {
      t.mainPhase = PHASE_WAIT_RESOURCE;
      return;
    }
    t.mainPhase = (t.currentTank < t.targetTank) ? PHASE_MOVE_RIGHT :
                  (t.currentTank > t.targetTank) ? PHASE_MOVE_LEFT : PHASE_PROCESS;
    t.phaseStartMs = millis();
  }
}

void handleMovePhase(uint8_t id) {
  TrolleyState& t = trolleyFor(id);
  const TrolleyPins& p = pinsFor(id);
  const bool movingRight = (t.mainPhase == PHASE_MOVE_RIGHT);

  if (isSharedTrackConflict(id, t.targetTank) || !isMotionDirectionSafe(id, movingRight)) {
    stopTrolleyOutputs(id);
    return;
  }

  const int remaining = abs(static_cast<int>(t.currentTank) - static_cast<int>(t.targetTank));
  const bool fastSegment = remaining > 1;
  digitalWrite(p.cmdFast, fastSegment ? HIGH : LOW);

  if (movingRight) {
    digitalWrite(p.cmdLeft, LOW);
    digitalWrite(p.cmdRight, HIGH);
  } else {
    digitalWrite(p.cmdRight, LOW);
    digitalWrite(p.cmdLeft, HIGH);
  }

  if (t.currentTank == t.targetTank && t.currentTank > 0) {
    stopTrolleyOutputs(id);
    t.mainPhase = PHASE_PROCESS;
    t.phaseStartMs = millis();
  }
}

void handleProcessCycle(uint8_t id) {
  TrolleyState& t = trolleyFor(id);
  const TrolleyPins& p = pinsFor(id);
  const unsigned long now = millis();

  switch (t.drumPhase) {
    case DRUM_IDLE:
      if (!isDrumLowerSafe(id)) {
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      if (tankOccupied[t.targetTank] && t.assignedPhysicalTank == 0) {
        t.mainPhase = PHASE_WAIT_RESOURCE;
        return;
      }
      if (t.assignedPhysicalTank == 0 && (t.logicalTarget >= 1 && t.logicalTarget <= TANK_COUNT)) {
        tankOccupied[t.targetTank] = true;
      }
      digitalWrite(p.cmdDrumDown, HIGH);
      t.drumPhase = DRUM_LOWERING;
      t.phaseStartMs = now;
      break;

    case DRUM_LOWERING:
      if (digitalRead(p.pinDLS) == LOW) {
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      if ((now - t.phaseStartMs) >= DRUM_ACTUATION_MS || digitalRead(p.pinDL) == LOW) {
        digitalWrite(p.cmdDrumDown, LOW);
        t.phaseStartMs = now;
        t.drumPhase = DRUM_WAIT_DT;
      }
      break;

    case DRUM_WAIT_DT: {
      // DT confirmation is used to trigger the immersion/dwell timer.
      const bool dtRaw = (digitalRead(p.pinDT) == LOW);
      const bool dtActive = DT_ACTIVE_LOW ? dtRaw : !dtRaw;

      if (dtActive) {
        t.processStartMs = now;
        t.drumPhase = DRUM_HOLDING;
      } else if ((now - t.phaseStartMs) >= DT_VERIFY_TIMEOUT_MS) {
        // DT not observed -> latch emergency to prevent unsafe/undefined immersion behavior.
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      break;
    }

    case DRUM_HOLDING:
      if ((now - t.processStartMs) >= getTaskDwellMs(id, t.taskPointer)) {
        // mark grouped bath as ready for later retrieval if this is a grouped process;
        // for fixed stations, raise immediately.
        if (t.logicalTarget == GROUP_ZINC || t.logicalTarget == GROUP_PASSIVATION) {
          tankReady[t.assignedPhysicalTank] = true;
          tankReadyTime[t.assignedPhysicalTank] = now;
          t.carryingDrum = false;
          t.drumPhase = DRUM_IDLE;
          t.taskPointer++;
          t.mainPhase = PHASE_DISPATCH;
        } else {
          if (!isDrumRaiseSafe(id)) {
            emergencyLatched = true;
            stopTrolleyOutputs(id);
            return;
          }
          digitalWrite(p.cmdDrumUp, HIGH);
          t.drumPhase = DRUM_RAISING;
          t.phaseStartMs = now;
        }
      }
      break;

    case DRUM_RAISING:
      if (digitalRead(p.pinDUS) == LOW) {
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      if ((now - t.phaseStartMs) >= DRUM_ACTUATION_MS || digitalRead(p.pinDU) == LOW) {
        digitalWrite(p.cmdDrumUp, LOW);
        releaseTank(t.targetTank);
        t.carryingDrum = true;
        t.drumPhase = DRUM_IDLE;
        t.taskPointer++;
        t.mainPhase = PHASE_DISPATCH;
      }
      break;
  }
}

void handleRetrieveGroup(uint8_t id) {
  TrolleyState& t = trolleyFor(id);
  const TrolleyPins& p = pinsFor(id);
  const unsigned long now = millis();

  if (t.currentTank != t.targetTank) {
    t.mainPhase = (t.currentTank < t.targetTank) ? PHASE_MOVE_RIGHT : PHASE_MOVE_LEFT;
    return;
  }

  switch (t.drumPhase) {
    case DRUM_IDLE:
      if (!isDrumRaiseSafe(id)) {
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      digitalWrite(p.cmdDrumUp, HIGH);       // retrieve completed drum
      t.drumPhase = DRUM_RAISING;
      t.phaseStartMs = now;
      break;

    case DRUM_RAISING:
      if (digitalRead(p.pinDUS) == LOW) {
        emergencyLatched = true;
        stopTrolleyOutputs(id);
        return;
      }
      if ((now - t.phaseStartMs) >= DRUM_ACTUATION_MS || digitalRead(p.pinDU) == LOW) {
        digitalWrite(p.cmdDrumUp, LOW);
        releaseTank(t.targetTank);
        t.carryingDrum = true;
        t.assignedPhysicalTank = 0;
        t.drumPhase = DRUM_IDLE;
        t.taskPointer++;
        t.mainPhase = PHASE_DISPATCH;
      }
      break;

    default:
      t.drumPhase = DRUM_IDLE;
      break;
  }
}

// -----------------------------------------------------------------------------
//  THERMAL CONTROL
// -----------------------------------------------------------------------------
void requestTemperaturesNonBlocking() {
  tempSensors.requestTemperatures();
  tempRequestInFlight = true;
  tempRequestStartedMs = millis();
  lastTempRequestMs = tempRequestStartedMs;
}

void readTemperaturesIfReady() {
  if (!tempRequestInFlight) return;
  if ((millis() - tempRequestStartedMs) < TEMP_CONVERSION_DELAY_MS) return;

  for (uint8_t i = 0; i < TEMP_ZONE_COUNT; ++i) {
    zoneTemps[i] = tempSensors.getTempCByIndex(i);
  }
  tempRequestInFlight = false;
}

void serviceThermalControl() {
  if (!tempRequestInFlight && (millis() - lastTempRequestMs >= TEMP_REQUEST_PERIOD_MS)) {
    requestTemperaturesNonBlocking();
  }
  readTemperaturesIfReady();

  for (uint8_t i = 0; i < TEMP_ZONE_COUNT; ++i) {
    const float t = zoneTemps[i];
    if (t < -50.0f || t > 125.0f) {
      digitalWrite(HEATER_PINS[i], LOW);
      if (i < 4) digitalWrite(FAN_PINS[i], LOW);
      continue;
    }

    if (t < (ZONE_SETPOINTS[i] - TEMP_HYST)) {
      digitalWrite(HEATER_PINS[i], HIGH);
      if (i < 4) digitalWrite(FAN_PINS[i], LOW);
    } else if (t > (ZONE_SETPOINTS[i] + TEMP_HYST)) {
      digitalWrite(HEATER_PINS[i], LOW);
      if (i < 4) digitalWrite(FAN_PINS[i], HIGH);
    }
  }
}

// -----------------------------------------------------------------------------
//  HMI
// -----------------------------------------------------------------------------
void serviceHMI() {
  const bool encA = digitalRead(ENC_A_PIN);
  const bool btn = digitalRead(ENC_SW_PIN);

  if (lastEncAState == HIGH && encA == LOW) {
    hmiPulseCounter++;
    menuIndex = static_cast<int>(hmiPulseCounter % 3L);
    if (menuIndex < 0) menuIndex += 3;
  }
  lastEncAState = encA;

  if (lastBtnState == HIGH && btn == LOW) {
    // Simple local action: cycle display pages only.
    hmiPulseCounter++;
    menuIndex = static_cast<int>(hmiPulseCounter % 3L);
  }
  lastBtnState = btn;

  if (millis() - lastHmiRefreshMs >= HMI_REFRESH_MS) {
    refreshLCD();
    lastHmiRefreshMs = millis();
  }
}

void refreshLCD() {
  lcd.clear();
  if (menuIndex == 0) {
    lcd.setCursor(0, 0); lcd.print("T1:"); lcd.print(T1.currentTank);
    lcd.setCursor(10, 0); lcd.print("P:"); lcd.print(T1.mainPhase);
    lcd.setCursor(0, 1); lcd.print("T2:"); lcd.print(T2.currentTank);
    lcd.setCursor(10, 1); lcd.print("P:"); lcd.print(T2.mainPhase);
    lcd.setCursor(0, 2); lcd.print("Run:"); lcd.print(systemRunning ? "YES" : "NO ");
    lcd.setCursor(0, 3); lcd.print("EMG:"); lcd.print(emergencyLatched ? "LATCH" : "OK   ");
  } else if (menuIndex == 1) {
    lcd.setCursor(0, 0); lcd.print("Zn grp ready:");
    lcd.setCursor(0, 1); lcd.print(selectReadyTankByTemporalFIFO(GROUP_ZINC));
    lcd.setCursor(0, 2); lcd.print("Pas ready:");
    lcd.setCursor(0, 3); lcd.print(selectReadyTankByTemporalFIFO(GROUP_PASSIVATION));
  } else {
    lcd.setCursor(0, 0); lcd.print("Temp1:"); lcd.print(zoneTemps[0]);
    lcd.setCursor(0, 1); lcd.print("Temp2:"); lcd.print(zoneTemps[1]);
    lcd.setCursor(0, 2); lcd.print("Temp3:"); lcd.print(zoneTemps[2]);
    lcd.setCursor(0, 3); lcd.print("Temp4:"); lcd.print(zoneTemps[3]);
  }
}

// -----------------------------------------------------------------------------
//  EMERGENCY HANDLING
// -----------------------------------------------------------------------------
void handleEmergencyStop() {
  if (emergencyLatched || digitalRead(EMG_STOP_PIN) == LOW) {
    emergencyLatched = true;
    stopTrolleyOutputs(0);
    stopTrolleyOutputs(1);
    for (uint8_t i = 0; i < TEMP_ZONE_COUNT; ++i) digitalWrite(HEATER_PINS[i], LOW);
    for (uint8_t i = 0; i < 4; ++i) digitalWrite(FAN_PINS[i], LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY STOP");
  }
}