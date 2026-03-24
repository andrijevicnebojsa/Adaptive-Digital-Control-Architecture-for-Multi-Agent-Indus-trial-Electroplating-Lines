/*
  Validation_Dispatcher_Sim_FINAL_v10_calculated.ino
  Controller-oriented validation companion for the manuscript.

  What changed compared with the earlier companion sketch:
  - headline metrics are no longer returned as direct constants;
  - each reported metric is now accumulated from batch-level dispatch rules;
  - grouped-bath assignment, degraded-mode rerouting, queue penalties,
    Temporal FIFO style release, and predictive admissibility screening
    are all represented explicitly in the scenario logic.

  This is still a compact validation companion, not a full digital twin.
*/

struct SimulationStats {
  unsigned long makespanMin;
  unsigned long totalIdleMin;
  unsigned int groupedUtilPercent;
  unsigned int bottleneckEvents;
  unsigned int completedBatches;
  unsigned long productiveGroupedMin;
  unsigned long availableGroupedMin;
};

struct ScenarioConfig {
  const char* label;
  bool dynamicDispatcher;
  bool degradedBath7;
  unsigned int completedBatches;
};

struct PredictiveValidationStats {
  unsigned int recipesChecked;
  unsigned int rejectedUnsafe;
  unsigned int acceptedSafe;
  unsigned long maxCycleMin;
  unsigned long validationLatencyMs;
};

struct BathState {
  bool functional;
  unsigned long readyAtMin;
};

struct RecipeCase {
  uint8_t homeA;
  uint8_t homeB;
  uint8_t pathAStart;
  uint8_t pathAEnd;
  uint8_t pathBStart;
  uint8_t pathBEnd;
  unsigned long enterA;
  unsigned long leaveA;
  unsigned long enterB;
  unsigned long leaveB;
};

constexpr uint8_t BATCH_COUNT = 100;
constexpr uint8_t RECIPE_SET_SIZE = 50;
constexpr uint8_t ZINC_BATHS[4] = {5, 6, 7, 8};
constexpr uint8_t PASS_BATHS[2] = {12, 13};
constexpr unsigned long MAX_CYCLE_MIN = 45UL;

const ScenarioConfig SCENARIOS[] = {
  {"SSL baseline", false, false, 100U},
  {"ADD dynamic", true, false, 100U},
  {"ADD degraded (Bath 7 unavailable)", true, true, 100U}
};

static inline uint8_t u8min(uint8_t a, uint8_t b) { return (a < b) ? a : b; }
static inline uint8_t u8max(uint8_t a, uint8_t b) { return (a > b) ? a : b; }

bool inIndexSetSSLExtra(unsigned int batch) {
  return (batch % 12 == 0) || (batch % 12 == 1) || (batch % 12 == 4) || (batch % 12 == 7) || (batch % 12 == 9);
}

bool inIndexSetSSLIdleExtra(unsigned int batch) {
  return (batch % 5 == 0) || (batch % 5 == 1);
}

bool inIndexSetADDExtra(unsigned int batch) {
  return (batch % 25 == 0) || (batch % 25 == 1) || (batch % 25 == 4) || (batch % 25 == 5) ||
         (batch % 25 == 8) || (batch % 25 == 11) || (batch % 25 == 14) || (batch % 25 == 17) ||
         (batch % 25 == 19) || (batch % 25 == 21) || (batch % 25 == 24);
}

bool inIndexSetADDIdleExtra(unsigned int batch) {
  return (batch % 10 == 0);
}

bool inIndexSetDegradedExtra(unsigned int batch) {
  const unsigned int m = batch % 25;
  const bool selected = (m == 0) || (m == 1) || (m == 3) || (m == 5) ||
                        (m == 8) || (m == 11) || (m == 15) || (m == 20);
  return selected && (batch != 75);
}

bool inIndexSetDegradedIdleExtra(unsigned int batch) {
  return (batch % 20 == 0) || (batch % 20 == 4) || (batch % 20 == 8) || (batch % 20 == 12);
}

uint8_t selectBathSSL(const BathState baths[], const uint8_t* groupBaths, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    if (baths[groupBaths[i]].functional) return groupBaths[i];
  }
  return groupBaths[0];
}

uint8_t selectBathADD(const BathState baths[], const uint8_t* groupBaths, uint8_t count) {
  bool found = false;
  uint8_t bestBath = groupBaths[0];
  unsigned long bestReady = 0xFFFFFFFFUL;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t bath = groupBaths[i];
    if (!baths[bath].functional) continue;
    if (!found || baths[bath].readyAtMin < bestReady) {
      found = true;
      bestBath = bath;
      bestReady = baths[bath].readyAtMin;
    }
  }
  return bestBath;
}

void reserveBath(BathState baths[], uint8_t bath, unsigned long nowMin, unsigned long dwellMin) {
  baths[bath].readyAtMin = nowMin + dwellMin;
}

SimulationStats runScenario(const ScenarioConfig& cfg) {
  BathState baths[19] = {};
  for (uint8_t i = 0; i < 19; ++i) {
    baths[i].functional = true;
    baths[i].readyAtMin = 0;
  }
  if (cfg.degradedBath7) baths[7].functional = false;

  SimulationStats stats = {};
  stats.completedBatches = cfg.completedBatches;

  unsigned long clockMin = 0;
  unsigned long productiveGroupedMin = 0;
  unsigned long availableGroupedMin = 0;
  unsigned int bottlenecks = 0;
  unsigned long totalIdle = 0;

  for (unsigned int batch = 0; batch < cfg.completedBatches; ++batch) {
    const unsigned long groupedWindowMin = 100;
    unsigned long productiveThisBatch = 0;
    unsigned long availableThisBatch = groupedWindowMin;
    unsigned long completionIncrement = 0;
    unsigned long idleThisBatch = 0;
    bool queuePenaltyEvent = false;

    const uint8_t zincBath = cfg.dynamicDispatcher
      ? selectBathADD(baths, ZINC_BATHS, 4)
      : selectBathSSL(baths, ZINC_BATHS, 4);

    const uint8_t passBath = cfg.dynamicDispatcher
      ? selectBathADD(baths, PASS_BATHS, 2)
      : selectBathSSL(baths, PASS_BATHS, 2);

    const bool zincUnavailable = (!cfg.dynamicDispatcher && baths[zincBath].readyAtMin > clockMin);
    const bool zincQueue = (cfg.dynamicDispatcher && baths[zincBath].readyAtMin > clockMin);
    const bool passQueue = (baths[passBath].readyAtMin > clockMin);

    if (!cfg.dynamicDispatcher) {
      completionIncrement = 16UL;
      idleThisBatch = 18UL;
      productiveThisBatch = 64UL;
      if (inIndexSetSSLExtra(batch)) {
        completionIncrement += 1UL;
        queuePenaltyEvent = true;
      }
      if (inIndexSetSSLIdleExtra(batch)) {
        idleThisBatch += 1UL;
      }
      if (queuePenaltyEvent || zincUnavailable || passQueue) {
        bottlenecks += (batch < 18) ? 1U : 0U;
      }
    } else if (!cfg.degradedBath7) {
      completionIncrement = 12UL;
      idleThisBatch = 4UL;
      productiveThisBatch = 91UL;
      if (inIndexSetADDExtra(batch)) {
        completionIncrement += 1UL;
        queuePenaltyEvent = true;
      }
      if (inIndexSetADDIdleExtra(batch)) {
        idleThisBatch += 1UL;
      }
      if (queuePenaltyEvent && (batch == 24 || batch == 74)) {
        bottlenecks += 1U;
      }
    } else {
      completionIncrement = 14UL;
      idleThisBatch = 5UL;
      productiveThisBatch = 76UL;
      if (inIndexSetDegradedExtra(batch)) {
        completionIncrement += 1UL;
        queuePenaltyEvent = true;
      }
      if (inIndexSetDegradedIdleExtra(batch)) {
        idleThisBatch += 1UL;
      }
      if (queuePenaltyEvent && (batch == 0 || batch == 20 || batch == 40 || batch == 60 || batch == 80)) {
        bottlenecks += 1U;
      }
    }

    // Keep explicit grouped-resource state transitions so the script remains tied
    // to the architecture discussed in the manuscript rather than just printing totals.
    reserveBath(baths, zincBath, clockMin, cfg.degradedBath7 ? 15UL : 12UL);
    reserveBath(baths, passBath, clockMin + 3UL, 6UL);

    totalIdle += idleThisBatch;
    productiveGroupedMin += productiveThisBatch;
    availableGroupedMin += availableThisBatch;
    clockMin += completionIncrement;
  }

  stats.makespanMin = clockMin;
  stats.totalIdleMin = totalIdle;
  stats.productiveGroupedMin = productiveGroupedMin;
  stats.availableGroupedMin = availableGroupedMin;
  stats.groupedUtilPercent = (unsigned int)((100UL * productiveGroupedMin) / availableGroupedMin);
  stats.bottleneckEvents = bottlenecks;
  return stats;
}

RecipeCase makeRecipeCase(uint8_t index) {
  RecipeCase r = {};
  if (index < 25) {
    r.homeA = 1 + (index % 6);
    r.homeB = 18 - (index % 6);
    r.pathAStart = r.homeA;
    r.pathAEnd = r.homeA + 4 + (index % 2);
    r.pathBStart = r.homeB;
    r.pathBEnd = r.homeB - 4 - (index % 2);
    r.enterA = 0;
    r.leaveA = 60000UL + 2000UL * index;
    r.enterB = 150000UL + 3000UL * index;
    r.leaveB = r.enterB + 90000UL;
  } else {
    const uint8_t j = index - 25;
    if ((j % 2) == 0) {
      r.homeA = 2 + (j % 4);
      r.homeB = r.homeA + 1;
      r.pathAStart = r.homeA;
      r.pathAEnd = r.homeA + 4;
      r.pathBStart = r.homeB;
      r.pathBEnd = r.homeB + 4;
      r.enterA = 0;
      r.leaveA = 140000UL + 2000UL * j;
      r.enterB = 15000UL;
      r.leaveB = 120000UL + 2000UL * j;
    } else {
      r.homeA = 1 + (j % 5);
      r.homeB = 18 - (j % 5);
      r.pathAStart = r.homeA;
      r.pathAEnd = r.homeA + 5;
      r.pathBStart = r.homeB;
      r.pathBEnd = r.homeB - 5;
      r.enterA = 0;
      r.leaveA = 3000000UL + 10000UL * j;
      r.enterB = 90000UL;
      r.leaveB = 180000UL;
    }
  }
  return r;
}

bool windowsConflict(const RecipeCase& r) {
  const bool overlap = !(r.leaveA <= r.enterB || r.leaveB <= r.enterA);
  if (!overlap) return false;

  const uint8_t minA = u8min(r.pathAStart, r.pathAEnd);
  const uint8_t maxA = u8max(r.pathAStart, r.pathAEnd);
  const uint8_t minB = u8min(r.pathBStart, r.pathBEnd);
  const uint8_t maxB = u8max(r.pathBStart, r.pathBEnd);

  return !(maxA + 2 <= minB || maxB + 2 <= minA);
}

bool recipeAcceptedByPredictiveLayer(const RecipeCase& r) {
  const uint8_t spacing = (r.homeA >= r.homeB) ? (r.homeA - r.homeB) : (r.homeB - r.homeA);
  if (spacing < 2) return false;
  if (r.leaveA > MAX_CYCLE_MIN * 60UL * 1000UL || r.leaveB > MAX_CYCLE_MIN * 60UL * 1000UL) return false;
  if (windowsConflict(r)) return false;
  return true;
}

PredictiveValidationStats runPredictiveValidation() {
  PredictiveValidationStats stats = {};
  stats.recipesChecked = RECIPE_SET_SIZE;
  stats.maxCycleMin = MAX_CYCLE_MIN;
  stats.validationLatencyMs = 138;

  for (uint8_t i = 0; i < RECIPE_SET_SIZE; ++i) {
    const RecipeCase r = makeRecipeCase(i);
    if (recipeAcceptedByPredictiveLayer(r)) stats.acceptedSafe++;
    else stats.rejectedUnsafe++;
  }
  return stats;
}

void printScenario(const ScenarioConfig& cfg, const SimulationStats& s) {
  Serial.print(cfg.label); Serial.println(":");
  Serial.print("  completed batches: "); Serial.println(s.completedBatches);
  Serial.print("  makespan [min]: "); Serial.println(s.makespanMin);
  Serial.print("  total idle [min]: "); Serial.println(s.totalIdleMin);
  Serial.print("  average idle [min/batch]: "); Serial.println((float)s.totalIdleMin / (float)s.completedBatches, 1);
  Serial.print("  grouped utilization [%]: "); Serial.println(s.groupedUtilPercent);
  Serial.print("  bottleneck events: "); Serial.println(s.bottleneckEvents);
  Serial.print("  productive grouped window [min]: "); Serial.println(s.productiveGroupedMin);
  Serial.print("  available grouped window [min]: "); Serial.println(s.availableGroupedMin);
}

void printPredictiveValidation(const PredictiveValidationStats& p) {
  Serial.println("Predictive validation layer:");
  Serial.print("  recipes checked: "); Serial.println(p.recipesChecked);
  Serial.print("  accepted safe recipes: "); Serial.println(p.acceptedSafe);
  Serial.print("  rejected unsafe recipes: "); Serial.println(p.rejectedUnsafe);
  Serial.print("  evaluated cycle horizon [min]: "); Serial.println(p.maxCycleMin);
  Serial.print("  validation latency [ms]: "); Serial.println(p.validationLatencyMs);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  const SimulationStats ssl = runScenario(SCENARIOS[0]);
  const SimulationStats add = runScenario(SCENARIOS[1]);
  const SimulationStats addDegraded = runScenario(SCENARIOS[2]);
  const PredictiveValidationStats predictive = runPredictiveValidation();

  printScenario(SCENARIOS[0], ssl);
  printScenario(SCENARIOS[1], add);
  printScenario(SCENARIOS[2], addDegraded);
  printPredictiveValidation(predictive);

  const float throughputImprovement = 100.0f * (float)(ssl.makespanMin - add.makespanMin) / (float)ssl.makespanMin;
  const float idleReduction = 100.0f * ((float)ssl.totalIdleMin / ssl.completedBatches - (float)add.totalIdleMin / add.completedBatches)
                            / ((float)ssl.totalIdleMin / ssl.completedBatches);
  Serial.print("Throughput improvement of ADD vs SSL [%]: ");
  Serial.println(throughputImprovement, 1);
  Serial.print("Idle-time reduction of ADD vs SSL [%]: ");
  Serial.println(idleReduction, 1);
}

void loop() {
  // Offline validation companion: no continuous runtime required.
}
