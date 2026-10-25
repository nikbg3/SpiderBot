#include <Servo.h>
#include <EEPROM.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <math.h>
#include <string.h>
#include "spiderbot_motion.h"
#include "webserver.h"

#ifdef ARDUINO_ESP32C3_DEV
const uint16_t kRecvPin = 10;
#else
const uint16_t kRecvPin = D3;
#endif

IRrecv irrecv(kRecvPin);
decode_results results;

uint32_t irnum = 0;

// Avoidance distance threshold in centimeters.
#define AVOIDDIS 20
#define ENABLE_DEBUG_SERIAL 0

// The current avoidance wiring uses TX as TRIG and RX as ECHO.
static const uint8_t ULTRASONIC_TRIG_PIN = 1;  // TX
static const uint8_t ULTRASONIC_ECHO_PIN = 3;  // RX
static const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

#if ENABLE_DEBUG_SERIAL
#define DEBUG_SERIAL_BEGIN(baud) Serial.begin(baud)
#define DEBUG_PRINT(value) Serial.print(value)
#define DEBUG_PRINTLN(value) Serial.println(value)
#define DEBUG_PRINT_HEX64(value) serialPrintUint64((value), HEX)
#else
#define DEBUG_SERIAL_BEGIN(baud) do {} while (0)
#define DEBUG_PRINT(value) do {} while (0)
#define DEBUG_PRINTLN(value) do {} while (0)
#define DEBUG_PRINT_HEX64(value) do {} while (0)
#endif

float distance = 0;

enum NavigationState {
  NAV_INITIALIZING,
  NAV_STARTUP_DELAY,
  NAV_INITIAL_SCANNING,
  NAV_EXPLORING,
  NAV_AVOIDING_OBSTACLE,
  NAV_REPOSITIONING,
  NAV_RECOVERING,
  NAV_PAUSED,
  NAV_STOPPED,
  NAV_ERROR
};

struct SensorScanSample {
  float headingDeg;
  float distanceCm;
  bool valid;
};

struct NavigationConfig {
  uint32_t startupDelayMs;
  float safeTurnDistanceCm;
  uint8_t maxRecoveryAttempts;
  uint8_t scanSteps;
};

static const NavigationConfig kDefaultNavConfig = {
  5000,
  30.0f,
  3,
  20
};

static const uint8_t kMapWidth = 31;
static const uint8_t kMapHeight = 31;
static const int kEstimatedTurnAngleDegrees = 18;
static const uint8_t kMaxBlockedCyclesBeforeRecovery = 5;
static const float kMapCellSizeCm = 10.0f;
static const uint8_t kMapOriginX = kMapWidth / 2;
static const uint8_t kMapOriginY = kMapHeight / 2;

enum MapCell : uint8_t {
  MAP_UNKNOWN = 0,
  MAP_FREE = 1,
  MAP_OBSTACLE = 2,
  MAP_PATH = 3,
  MAP_ROBOT = 4
};

static NavigationConfig g_navigationConfig = kDefaultNavConfig;
static NavigationState g_navigationState = NAV_INITIALIZING;
static bool g_autonomousEnabled = false;
static bool g_initialScanComplete = false;
static uint32_t g_startupDeadlineMs = 0;
static int g_explorationDirection = 1;
static int g_recoveryAttempts = 0;
static uint8_t g_blockedCycles = 0;
static uint32_t g_lastTelemetryPublishMs = 0;
static int g_estimatedHeadingDegrees = 0;
static float g_distanceTravelledCm = 0.0f;
static float g_lastMovementCm = 0.0f;
static float g_currentDistanceCm = -1.0f;
static bool g_sensorValid = false;
static uint8_t g_occupancyGrid[kMapWidth * kMapHeight] = {0};
static uint8_t g_shiftedOccupancyGrid[kMapWidth * kMapHeight];
static float g_mapPositionX = kMapOriginX;
static float g_mapPositionY = kMapOriginY;
static SensorScanSample g_scanSamples[36];
static int g_scanSampleCount = 0;
static uint16_t g_scanDistancesCm[12] = {0};

// Main robot motion object plus a few local helpers used by setup/loop.
SpiderBotMotion robot;
static int decodeIrProgram(uint32_t code);
static void runRobotProgram(int programId);
static float measureAvoidDistanceCm();
static bool verifyRobotHardware();
static const char* navigationStateName(NavigationState state);
static void logNavigationEvent(const char* message);
static bool performInitialScan();
static void updateAutonomousNavigation();
static int chooseSafeTurnDirection();
static void triggerRecovery();
static void publishNavigationStatus();
static uint16_t countMappedCells();
static bool mapCellInBounds(int x, int y);
static void markMapCell(int x, int y, MapCell cell);
static void recenterMapForPosition(float& x, float& y);
static void mapRangeMeasurement(float distanceCm, float headingDegrees);
static void recordEstimatedMovement(float distanceCm);

static const char* navigationStateName(NavigationState state)
{
  switch (state) {
    case NAV_INITIALIZING: return "INITIALIZING";
    case NAV_STARTUP_DELAY: return "STARTUP_DELAY";
    case NAV_INITIAL_SCANNING: return "INITIAL_SCANNING";
    case NAV_EXPLORING: return "EXPLORING";
    case NAV_AVOIDING_OBSTACLE: return "AVOIDING_OBSTACLE";
    case NAV_REPOSITIONING: return "REPOSITIONING";
    case NAV_RECOVERING: return "RECOVERING";
    case NAV_PAUSED: return "PAUSED";
    case NAV_STOPPED: return "STOPPED";
    case NAV_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

static void logNavigationEvent(const char* message)
{
  if (ENABLE_DEBUG_SERIAL) {
    Serial.print(millis());
    Serial.print(" ms | ");
    Serial.print(navigationStateName(g_navigationState));
    Serial.print(" | ");
    Serial.println(message);
  }
}

static bool verifyRobotHardware()
{
  float sample = measureAvoidDistanceCm();
  if (sample < 0.0f) {
    logNavigationEvent("sensor initialization failed");
    return false;
  }

  return true;
}

static void publishNavigationStatus()
{
  const uint32_t nowMs = millis();
  if (nowMs - g_lastTelemetryPublishMs < 500) {
    return;
  }

  publishNavigationTelemetry(
    navigationStateName(g_navigationState),
    g_currentDistanceCm,
    g_distanceTravelledCm,
    (float)countMappedCells() * kMapCellSizeCm * kMapCellSizeCm / 10000.0f,
    g_estimatedHeadingDegrees,
    g_recoveryAttempts,
    g_sensorValid,
    g_scanDistancesCm,
    g_occupancyGrid);
  g_lastTelemetryPublishMs = nowMs;
}

static uint16_t countMappedCells()
{
  uint16_t count = 0;
  for (size_t i = 0; i < sizeof(g_occupancyGrid); ++i) {
    if (g_occupancyGrid[i] != MAP_UNKNOWN) {
      ++count;
    }
  }
  return count;
}

static bool mapCellInBounds(int x, int y)
{
  return x >= 0 && x < kMapWidth && y >= 0 && y < kMapHeight;
}

static void markMapCell(int x, int y, MapCell cell)
{
  if (!mapCellInBounds(x, y)) {
    return;
  }

  uint8_t& current = g_occupancyGrid[y * kMapWidth + x];
  if (cell == MAP_ROBOT || cell == MAP_OBSTACLE ||
      (cell == MAP_PATH && current != MAP_OBSTACLE) ||
      (cell == MAP_FREE && current == MAP_UNKNOWN)) {
    current = cell;
  }
}

static void recenterMapForPosition(float& x, float& y)
{
  int shiftX = 0;
  int shiftY = 0;
  const int margin = 3;

  if (x < margin || x >= kMapWidth - margin) {
    shiftX = (int)lroundf(x - kMapOriginX);
  }
  if (y < margin || y >= kMapHeight - margin) {
    shiftY = (int)lroundf(y - kMapOriginY);
  }
  if (shiftX == 0 && shiftY == 0) {
    return;
  }

  memset(g_shiftedOccupancyGrid, MAP_UNKNOWN, sizeof(g_shiftedOccupancyGrid));
  for (int oldY = 0; oldY < kMapHeight; ++oldY) {
    for (int oldX = 0; oldX < kMapWidth; ++oldX) {
      const int newX = oldX - shiftX;
      const int newY = oldY - shiftY;
      if (mapCellInBounds(newX, newY)) {
        g_shiftedOccupancyGrid[newY * kMapWidth + newX] =
            g_occupancyGrid[oldY * kMapWidth + oldX];
      }
    }
  }

  memcpy(g_occupancyGrid, g_shiftedOccupancyGrid, sizeof(g_occupancyGrid));
  g_mapPositionX -= shiftX;
  g_mapPositionY -= shiftY;
  x -= shiftX;
  y -= shiftY;
}

static void mapRangeMeasurement(float distanceCm, float headingDegrees)
{
  if (distanceCm <= 0.0f || distanceCm >= 400.0f) {
    return;
  }

  const float radians = headingDegrees * 0.01745329252f;
  const float directionX = sinf(radians);
  const float directionY = -cosf(radians);
  const float halfCellCm = kMapCellSizeCm * 0.5f;

  for (float traveledCm = halfCellCm;
       traveledCm < distanceCm;
       traveledCm += halfCellCm) {
    const int x = (int)lroundf(g_mapPositionX + directionX * traveledCm / kMapCellSizeCm);
    const int y = (int)lroundf(g_mapPositionY + directionY * traveledCm / kMapCellSizeCm);
    if (!mapCellInBounds(x, y)) {
      break;
    }
    markMapCell(x, y, MAP_FREE);
  }

  const int obstacleX = (int)lroundf(g_mapPositionX + directionX * distanceCm / kMapCellSizeCm);
  const int obstacleY = (int)lroundf(g_mapPositionY + directionY * distanceCm / kMapCellSizeCm);
  markMapCell(obstacleX, obstacleY, MAP_OBSTACLE);
}

static void recordEstimatedMovement(float distanceCm)
{
  if (distanceCm == 0.0f) {
    return;
  }

  const float radians = g_estimatedHeadingDegrees * 0.01745329252f;
  const float startX = g_mapPositionX;
  const float startY = g_mapPositionY;
  const float targetX = startX + sinf(radians) * distanceCm / kMapCellSizeCm;
  const float targetY = startY - cosf(radians) * distanceCm / kMapCellSizeCm;
  float endX = targetX;
  float endY = targetY;
  const float steps = fabsf(distanceCm) / (kMapCellSizeCm * 0.5f);

  markMapCell((int)lroundf(startX), (int)lroundf(startY), MAP_PATH);
  recenterMapForPosition(endX, endY);
  const float adjustedStartX = startX + endX - targetX;
  const float adjustedStartY = startY + endY - targetY;
  for (int step = 1; step <= (int)steps; ++step) {
    const float fraction = (float)step / steps;
    markMapCell((int)lroundf(adjustedStartX + (endX - adjustedStartX) * fraction),
        (int)lroundf(adjustedStartY + (endY - adjustedStartY) * fraction), MAP_PATH);
  }

  g_mapPositionX = endX;
  g_mapPositionY = endY;
  markMapCell((int)lroundf(g_mapPositionX), (int)lroundf(g_mapPositionY), MAP_ROBOT);
}

static int chooseSafeTurnDirection()
{
  float leftScore = 0.0f;
  float rightScore = 0.0f;

  for (int i = 0; i < g_scanSampleCount; ++i) {
    const SensorScanSample& sample = g_scanSamples[i];
    if (!sample.valid) {
      continue;
    }

    if (sample.headingDeg >= 0.0f && sample.headingDeg <= 180.0f) {
      leftScore += sample.distanceCm;
    } else {
      rightScore += sample.distanceCm;
    }
  }

  if (leftScore > rightScore) {
    return 1;
  }
  if (rightScore > leftScore) {
    return -1;
  }
  return g_explorationDirection;
}

static bool performInitialScan()
{
  g_scanSampleCount = 0;
  memset(g_scanDistancesCm, 0, sizeof(g_scanDistancesCm));
  float leftScore = 0.0f;
  float rightScore = 0.0f;
  int scanSteps = min((int)g_navigationConfig.scanSteps, 36);

  logNavigationEvent("starting 360-degree scan");
  for (int i = 0; i < scanSteps; ++i) {
    float heading = (float)i * (360.0f / scanSteps);
    g_estimatedHeadingDegrees = (int)heading;
    float currentDistance = measureAvoidDistanceCm();
    SensorScanSample sample;
    sample.headingDeg = heading;
    sample.distanceCm = currentDistance;
    sample.valid = (currentDistance > 0.0f && currentDistance < 400.0f);

    if (sample.valid) {
      g_scanSamples[g_scanSampleCount++] = sample;
      int mapSector = ((int)(heading / 30.0f)) % 12;
      g_scanDistancesCm[mapSector] = (uint16_t)min((int)currentDistance, 400);
      mapRangeMeasurement(currentDistance, heading);
      if (sample.headingDeg <= 180.0f) {
        leftScore += sample.distanceCm;
      } else {
        rightScore += sample.distanceCm;
      }
    }

    if (i + 1 < scanSteps) {
      robot.turnleft();
      g_estimatedHeadingDegrees = ((int)((i + 1) * (360.0f / scanSteps))) % 360;
      delay(120);
    }
  }

  robot.center();
  delay(150);

  g_initialScanComplete = (g_scanSampleCount > 0);
  if (!g_initialScanComplete) {
    g_navigationState = NAV_ERROR;
    logNavigationEvent("initial scan produced no usable samples");
    return false;
  }

  g_explorationDirection = (leftScore >= rightScore) ? 1 : -1;
  g_estimatedHeadingDegrees = 0;
  markMapCell((int)lroundf(g_mapPositionX), (int)lroundf(g_mapPositionY), MAP_ROBOT);
  g_navigationState = NAV_EXPLORING;
  logNavigationEvent("initial scan complete");
  return true;
}

static void triggerRecovery()
{
  if (g_recoveryAttempts >= g_navigationConfig.maxRecoveryAttempts) {
    g_navigationState = NAV_STOPPED;
    requestRobotStop(PROGRAM_STANDBY);
    logNavigationEvent("recovery attempts exhausted; robot stopped");
    return;
  }

  g_navigationState = NAV_RECOVERING;
  g_recoveryAttempts++;
  logNavigationEvent("repeated blocked path; attempting recovery");

  robot.center();
  delay(150);

  const int turnDirection = -g_explorationDirection;
  if (turnDirection > 0) {
    robot.turnleft();
    g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + kEstimatedTurnAngleDegrees) % 360;
  } else {
    robot.turnright();
    g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + 360 - kEstimatedTurnAngleDegrees) % 360;
  }

  g_explorationDirection = turnDirection;
  g_blockedCycles = 0;
  g_navigationState = NAV_EXPLORING;
  logNavigationEvent("recovery heading selected");
}

static void updateAutonomousNavigation()
{
  if (g_navigationState == NAV_STARTUP_DELAY) {
    if (millis() >= g_startupDeadlineMs) {
      g_autonomousEnabled = true;
      g_navigationState = NAV_INITIAL_SCANNING;
      logNavigationEvent("startup delay complete; autonomous navigation enabled");
    }
    return;
  }

  if (!g_autonomousEnabled || isRobotStopRequested()) {
    g_navigationState = NAV_PAUSED;
    return;
  }

  if (g_navigationState == NAV_INITIAL_SCANNING) {
    performInitialScan();
    return;
  }

  if (g_navigationState == NAV_EXPLORING) {
    float obstacleDistance = measureAvoidDistanceCm();
    if (!g_sensorValid || obstacleDistance <= 0.0f) {
      robot.center();
      return;
    }

    if (obstacleDistance < g_navigationConfig.safeTurnDistanceCm) {
      if (g_blockedCycles < UINT8_MAX) {
        ++g_blockedCycles;
      }
      if (g_blockedCycles >= kMaxBlockedCyclesBeforeRecovery) {
        triggerRecovery();
      } else {
        g_navigationState = NAV_AVOIDING_OBSTACLE;
        logNavigationEvent("obstacle ahead; changing heading");
      }
      return;
    }

    g_blockedCycles = 0;
    g_recoveryAttempts = 0;

    robot.forward();
    recordEstimatedMovement(8.0f);
    g_distanceTravelledCm += 8.0f;
    g_lastMovementCm = 8.0f;

    static uint8_t sweepCounter = 0;
    ++sweepCounter;
    if (sweepCounter >= 3) {
      sweepCounter = 0;
      g_navigationState = NAV_REPOSITIONING;
    }
    return;
  }

  if (g_navigationState == NAV_AVOIDING_OBSTACLE) {
    robot.center();
    delay(120);
    if (chooseSafeTurnDirection() > 0) {
      robot.turnleft();
      g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + kEstimatedTurnAngleDegrees) % 360;
    } else {
      robot.turnright();
      g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + 360 - kEstimatedTurnAngleDegrees) % 360;
    }
    g_navigationState = NAV_EXPLORING;
    return;
  }

  if (g_navigationState == NAV_REPOSITIONING) {
    if (g_explorationDirection > 0) {
      robot.turnleft();
      g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + kEstimatedTurnAngleDegrees) % 360;
    } else {
      robot.turnright();
      g_estimatedHeadingDegrees = (g_estimatedHeadingDegrees + 360 - kEstimatedTurnAngleDegrees) % 360;
    }
    g_explorationDirection *= -1;
    g_navigationState = NAV_EXPLORING;
    return;
  }

  if (g_navigationState == NAV_RECOVERING) {
    g_navigationState = NAV_EXPLORING;
    return;
  }
}

// Board startup order:
// 1. bring up debug / IR / ultrasonic pins
// 2. start Wi-Fi + web server
// 3. attach servos and move to a known pose
void setup()
{
  DEBUG_SERIAL_BEGIN(9600);
  irrecv.enableIRIn();
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  DEBUG_PRINTLN("QuadBot-E Start!");
  delay(1000);

  const bool wifiStarted = webinit();
  enableWebServer();

  robot.init();
  robot.Servo_PROGRAM_Zero();
  markMapCell((int)lroundf(g_mapPositionX), (int)lroundf(g_mapPositionY), MAP_ROBOT);

  if (!wifiStarted) {
    g_navigationState = NAV_ERROR;
    g_autonomousEnabled = false;
    logNavigationEvent("Wi-Fi access point failed to start; autonomous navigation disabled");
    return;
  }

  if (!verifyRobotHardware()) {
    g_navigationState = NAV_ERROR;
    g_autonomousEnabled = false;
    logNavigationEvent("sensor unavailable; autonomous navigation disabled; manual control remains enabled");
    return;
  }

  g_navigationState = NAV_STARTUP_DELAY;
  g_startupDeadlineMs = millis() + g_navigationConfig.startupDelayMs;
  g_autonomousEnabled = false;
  logNavigationEvent("startup delay started");
}

// Main loop responsibilities:
// 1. consume IR commands
// 2. serve HTTP requests
// 3. execute queued web/API actions
// 4. apply direct single-servo debug commands
void loop()
{
  if (irrecv.decode(&results)) {
    int programId = PROGRAM_NONE;

    DEBUG_PRINT_HEX64(results.value);
    irnum = (uint32_t)results.value;
    delay(150);
    irrecv.resume();

    programId = decodeIrProgram(irnum);
    if (programId != PROGRAM_NONE) {
      DEBUG_PRINTLN(getActionLabel(programId));
      g_autonomousEnabled = false;
      g_navigationState = NAV_PAUSED;
      runRobotProgram(programId);
    }

    DEBUG_PRINTLN(irnum);
    irnum = 0;
  }

  handleClient();

  if (isRobotStopRequested() && !isRobotBusy()) {
    const int stopTargetProgram = getRobotStopTargetProgram();
    clearRobotStopRequest();
    g_autonomousEnabled = false;
    if (g_navigationState != NAV_STOPPED) {
      g_navigationState = NAV_PAUSED;
    }

    if (stopTargetProgram == PROGRAM_CENTER) {
      robot.center();
    } else if (stopTargetProgram == PROGRAM_ZERO) {
      robot.zero();
    } else {
      robot.standby();
    }
  }

  if (Servo_PROGRAM >= 1) {
    DEBUG_PRINTLN(getActionLabel(Servo_PROGRAM));
    g_autonomousEnabled = false;
    g_navigationState = NAV_PAUSED;
    const int requestedProgram = Servo_PROGRAM;
    runRobotProgram(requestedProgram);
    if (Servo_PROGRAM == requestedProgram) {
      Servo_PROGRAM = 0;
    }
  }

  if (GPIO_ID < 100) {
    DEBUG_PRINT("GPIO_ID=");
    DEBUG_PRINTLN(GPIO_ID);
    DEBUG_PRINT("ival=");
    DEBUG_PRINTLN(ival);
    robot.Set_PWM_to_Servo(GPIO_ID, ival.toInt());
    GPIO_ID = 100;
    ival = "";
  }

  if (g_navigationState != NAV_PAUSED && g_navigationState != NAV_STOPPED && g_navigationState != NAV_ERROR) {
    updateAutonomousNavigation();
  }
  publishNavigationStatus();
}

// Trigger a standard HC-SR04 style measurement and convert it to centimeters.
// Returns -1 when no echo is received before timeout.
static float measureAvoidDistanceCm()
{
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  const unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    g_currentDistanceCm = -1.0f;
    g_sensorValid = false;
    return -1.0f;
  }

  const float measuredDistanceCm = durationUs / 58.0f;
  g_currentDistanceCm = measuredDistanceCm;
  g_sensorValid = (measuredDistanceCm > 0.0f && measuredDistanceCm < 400.0f);
  if (g_sensorValid && g_navigationState == NAV_EXPLORING) {
    mapRangeMeasurement(measuredDistanceCm, g_estimatedHeadingDegrees);
  }
  if (g_sensorValid && g_initialScanComplete) {
    int mapSector = ((g_estimatedHeadingDegrees + 15) / 30) % 12;
    g_scanDistancesCm[mapSector] = (uint16_t)min((int)measuredDistanceCm, 400);
  }
  return measuredDistanceCm;
}

// Blocking avoidance mode:
// keep moving forward while the path is clear, otherwise stop and turn left twice.
// Stop requests and any other remote or web action interrupt the cycle.
void Avoid(void)
{
  while (1) {
    distance = measureAvoidDistanceCm();

    if (distance < AVOIDDIS && distance > 0) {
      robot.center();
      delay(100);
      robot.turnleft();
      robot.turnleft();
    } else if (distance >= AVOIDDIS) {
      robot.forward();
    }

    handleClient();

    if (isRobotStopRequested()) {
      break;
    }

    if (Servo_PROGRAM >= 1 && Servo_PROGRAM != PROGRAM_AVOID) {
      break;
    }

    if (irrecv.decode(&results)) {
      DEBUG_PRINT_HEX64(results.value);
      irnum = (uint32_t)results.value;
      irrecv.resume();
      const int requestedProgram = decodeIrProgram(irnum);
      if (requestedProgram != PROGRAM_NONE && requestedProgram != PROGRAM_AVOID) {
        Servo_PROGRAM = requestedProgram;
        break;
      }
    }

    if (Servo_PROGRAM == 22) {
      break;
    }
  }
}


// Legacy mapping kept here as a reference for the older unnamed IR remote.
/* static int decodeIrProgram(uint32_t code)
{
  switch (code)
  {
    case 0xFF18E7:
      return PROGRAM_FORWARD; // 上
    case 0xFF4AB5:
      return PROGRAM_BACKWARD; // 下
    case 0xFF10EF:
      return PROGRAM_LEFTSHIFT; // 左
    case 0xFF5AA5:
      return PROGRAM_RIGHTSHIFT; // 右
    case 0xFF38C7:
      return PROGRAM_ZERO; // OK
    case 0xFFB04F:
      return PROGRAM_CENTER; // #
    case 0xFF6897:
      return PROGRAM_SLEEP; //  *
    case 0xFFA25D:
      return PROGRAM_TURNLEFT; // 1
    case 0xFF629D:
      return PROGRAM_TURNRIGHT; // 2
    case 0xFFE21D:
      return PROGRAM_LIE; // 3
    case 0xFF22DD:
      return PROGRAM_HELLO; // 4
    case 0xFF02FD:
      return PROGRAM_FIGHTING; // 5
    case 0xFFC23D:
      return PROGRAM_PUSHUP; // 6
    case 0xFFE01F:
      return PROGRAM_DANCE1; // 7
    case 0xFFA857:
      return PROGRAM_DANCE2; // 8
    case 0xFF906F:
      return PROGRAM_DANCE3; // 9
    case 0xFF9867:
      return PROGRAM_AVOID; // 0
    default:
      return PROGRAM_NONE;
  }
} */
// Active IR mapping for the current remote controller.
// This converts raw IR values into the shared RobotProgramId enum.
static int decodeIrProgram(uint32_t code)
{
  switch (code)
  {
    case 0xFF629D:
      return PROGRAM_FORWARD;// ok 
    case 0xFFA857:
      return PROGRAM_BACKWARD;// ok
    case 0xFF22DD:
      return PROGRAM_LEFTSHIFT;// ok
    case 0xFFC23D:
      return PROGRAM_RIGHTSHIFT;// ok
    case 0xFF02FD:
      return PROGRAM_ZERO;// ok
    case 0xFF52AD:
      return PROGRAM_CENTER;// ok
    case 0xFF42BD:
      return PROGRAM_SLEEP;// ok
    case 0xFF6897:
      return PROGRAM_TURNLEFT; // ok
    case 0xFF9867:
      return PROGRAM_TURNRIGHT; // ok
    case 0xFFB04F:
      return PROGRAM_LIE;// ok
    case 0xFF30CF:
      return PROGRAM_HELLO;// ok
    case 0xFF18E7:
      return PROGRAM_FIGHTING;// ok
    case 0xFF7A85:
      return PROGRAM_PUSHUP;// ok
    case 0xFF10EF:
      return PROGRAM_DANCE1;// ok
    case 0xFF38C7:
      return PROGRAM_DANCE2;// ok
    case 0xFF5AA5:
      return PROGRAM_DANCE3;// ok
    case 0xFF4AB5:
      return PROGRAM_AVOID;
    default:
      return PROGRAM_NONE;
  }
}

// Single dispatch point used by IR, Web UI and API commands.
// Every control source eventually resolves to one RobotProgramId and lands here.
static void runRobotProgram(int programId)
{
  setCurrentAction(programId, true);

  switch (programId)
  {
    case PROGRAM_STANDBY:
      robot.standby();
      break;
    case PROGRAM_FORWARD:
      robot.forward();
      break;
    case PROGRAM_BACKWARD:
      robot.backward();
      break;
    case PROGRAM_LEFTSHIFT:
      robot.leftshift();
      break;
    case PROGRAM_RIGHTSHIFT:
      robot.rightshift();
      break;
    case PROGRAM_TURNLEFT:
      robot.turnleft();
      break;
    case PROGRAM_TURNRIGHT:
      robot.turnright();
      break;
    case PROGRAM_LIE:
      robot.lie();
      break;
    case PROGRAM_HELLO:
      robot.hello();
      break;
    case PROGRAM_FIGHTING:
      robot.fighting();
      break;
    case PROGRAM_PUSHUP:
      robot.pushup();
      break;
    case PROGRAM_SLEEP:
      robot.sleep();
      break;
    case PROGRAM_DANCE1:
      robot.dance1();
      break;
    case PROGRAM_DANCE2:
      robot.dance2();
      break;
    case PROGRAM_DANCE3:
      robot.dance3();
      break;
    case PROGRAM_AVOID:
      Avoid();
      break;
    case PROGRAM_CENTER:
      robot.center();
      break;
    case PROGRAM_ZERO:
      robot.zero();
      break;
    default:
      break;
  }

  if (isRobotStopRequested()) {
    int stopTargetProgram = getRobotStopTargetProgram();
    clearRobotStopRequest();

    if (stopTargetProgram == PROGRAM_CENTER) {
      robot.center();
    } else if (stopTargetProgram == PROGRAM_ZERO) {
      robot.zero();
    } else {
      robot.standby();
    }
  }

  clearCurrentAction();
}
