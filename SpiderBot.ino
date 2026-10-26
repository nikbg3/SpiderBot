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
  NAV_TARGETING,
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
};

static const float kForwardStepDistanceCm = 8.0f;
static const float kTurnAngleDegrees = 18.70f;

static const NavigationConfig kDefaultNavConfig = {
  5000,
  30.0f,
  3
};

static const uint8_t kMapWidth = NAVIGATION_MAP_MAX_WIDTH;
static const uint8_t kMapHeight = NAVIGATION_MAP_MAX_WIDTH;
static const uint8_t kLaneSpacingSteps = 1;
static const uint8_t kMaxBlockedCyclesBeforeRecovery = 5;
static const uint8_t kMaxInvalidSensorCycles = 5;
static const uint8_t kSensorRecoveryReadAttempts = 3;
static const uint16_t kSensorRecoveryRetryDelayMs = 80;
static const float kMapCellSizeCm = 10.0f;
static const uint8_t kMapOriginX = kMapWidth / 2;
static const uint8_t kMapOriginY = kMapHeight / 2;
static const float kSensorMaximumRangeCm = 400.0f;
static const float kUltrasonicBeamHalfAngleRadians = 0.130899694f;

enum MapCell : uint8_t {
  MAP_UNKNOWN = 0,
  MAP_FREE = 1,
  MAP_OBSTACLE = 2,
  MAP_PATH = 3,
  MAP_ROBOT = 4
};

enum SensorRecoveryResult : uint8_t {
  SENSOR_RECOVERED,
  SENSOR_RECOVERY_FAILED,
  SENSOR_RECOVERY_INTERRUPTED
};

static NavigationConfig g_navigationConfig = kDefaultNavConfig;
static NavigationState g_navigationState = NAV_INITIALIZING;
static bool g_autonomousEnabled = false;
static bool g_initialScanComplete = false;
static uint32_t g_startupDeadlineMs = 0;
static int g_laneChangeDirection = 1;
static int g_recoveryAttempts = 0;
static uint8_t g_blockedCycles = 0;
static uint8_t g_invalidSensorCycles = 0;
static uint32_t g_lastTelemetryPublishMs = 0;
static char g_navigationReason[96] = "Starting up";
static float g_estimatedHeadingDegrees = 0.0f;
static float g_distanceTravelledCm = 0.0f;
static float g_lastMovementCm = 0.0f;
static float g_currentDistanceCm = -1.0f;
static bool g_sensorValid = false;
static uint32_t g_nextWifiRetryMs = 0;
static uint8_t g_occupancyGrid[kMapWidth * kMapHeight] = {0};
static uint8_t g_shiftedOccupancyGrid[kMapWidth * kMapHeight];
static float g_mapPositionX = kMapOriginX;
static float g_mapPositionY = kMapOriginY;
static int g_mapOffsetX = -kMapOriginX;
static int g_mapOffsetY = -kMapOriginY;
static float g_worldPositionX = 0.0f;
static float g_worldPositionY = 0.0f;
static NavigationWaypoint g_activeWaypoints[MAX_NAVIGATION_WAYPOINTS];
static uint8_t g_activeWaypointCount = 0;
static uint8_t g_activeWaypointIndex = 0;
static bool g_targetNavigationActive = false;
static NavigationTargetKind g_activeTargetKind = NAV_TARGET_NONE;
static bool g_targetDetourActive = false;
static float g_targetDetourHeading = 0.0f;
static SensorScanSample g_scanSamples[160];
static int g_scanSampleCount = 0;
static uint16_t g_scanDistancesCm[12] = {0};
static bool g_scanInProgress = false;
static NavigationState g_scanReturnState = NAV_INITIAL_SCANNING;
static float g_scanStartHeadingDegrees = 0.0f;
static uint8_t g_scanNextStep = 0;
static uint8_t g_scanStepCount = 0;
static float g_scanStepCommandDegrees = kTurnAngleDegrees;
static float g_scanCommandDegrees = 360.0f;
static float g_scanDegreesPerCommandDegree = 1.0f;
static float g_scanLeftScore = 0.0f;
static float g_scanRightScore = 0.0f;
static int g_scanLeftCount = 0;
static int g_scanRightCount = 0;

// Main robot motion object plus a few local helpers used by setup/loop.
SpiderBotMotion robot;
static int decodeIrProgram(uint32_t code);
static void runRobotProgram(int programId);
static float measureAvoidDistanceCm();
static bool verifyRobotHardware();
static SensorRecoveryResult recoverUltrasonicSensor();
static const char* navigationStateName(NavigationState state);
static void logNavigationEvent(const char* message);
static bool perform360Scan();
static void updateAutonomousNavigation();
static void processNavigationControl();
static void processNavigationTarget();
static void updateTargetNavigation();
static bool rotateQuarterTurn(int direction);
static bool rotateByDegrees(int direction, float degrees,
    int executingProgramId = PROGRAM_NONE);
static bool performTurnStep(int direction, float degrees,
    float estimatedDegrees = -1.0f,
    int executingProgramId = PROGRAM_NONE);
static bool moveForwardSteps(uint8_t steps, bool checkObstacles = false);
static bool changeLane(int direction, bool reverseFirst);
static int chooseSafeTurnDirection();
static void triggerRecovery();
static void stopCoverageAtBoundary();
static void publishNavigationStatus(bool force = false);
static uint16_t countMappedCells();
static bool mapCellInBounds(int x, int y);
static void markMapCell(int x, int y, MapCell cell);
static void recenterMapForPosition(float& x, float& y);
static void mapRangeMeasurement(float distanceCm, float headingDegrees);
static void recordEstimatedMovement(float distanceCm);
static float normalizeHeadingDegrees(float headingDegrees);
static float signedHeadingErrorDegrees(float targetDegrees, float currentDegrees);

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
    case NAV_TARGETING: return "TARGETING";
    case NAV_PAUSED: return "PAUSED";
    case NAV_STOPPED: return "STOPPED";
    case NAV_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

static void logNavigationEvent(const char* message)
{
  strncpy(g_navigationReason, message, sizeof(g_navigationReason) - 1);
  g_navigationReason[sizeof(g_navigationReason) - 1] = '\0';
  publishNavigationStatus(true);
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
  measureAvoidDistanceCm();
  if (g_sensorValid) {
    return true;
  }

  logNavigationEvent("sensor verification failed; retrying sensor");
  return recoverUltrasonicSensor() == SENSOR_RECOVERED;
}

static SensorRecoveryResult recoverUltrasonicSensor()
{
  const NavigationState previousState = g_navigationState;
  g_navigationState = NAV_RECOVERING;
  logNavigationEvent("ultrasonic sensor readings are invalid; retrying while stationary");
  robot.center();

  for (uint8_t attempt = 0; attempt < kSensorRecoveryReadAttempts; ++attempt) {
    if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
        hasNavigationControlPending() || hasNavigationTargetPending()) {
      g_navigationState = NAV_PAUSED;
      return SENSOR_RECOVERY_INTERRUPTED;
    }

    if (measureAvoidDistanceCm() >= 0.0f && g_sensorValid) {
      g_invalidSensorCycles = 0;
      g_navigationState = previousState;
      logNavigationEvent("ultrasonic sensor recovered");
      return SENSOR_RECOVERED;
    }

    if (attempt + 1 < kSensorRecoveryReadAttempts) {
      delay(kSensorRecoveryRetryDelayMs);
      handleClient();
    }
  }

  g_navigationState = previousState;
  logNavigationEvent("ultrasonic sensor recovery retries exhausted");
  return SENSOR_RECOVERY_FAILED;
}

static void publishNavigationStatus(bool force)
{
  const uint32_t nowMs = millis();
  if (!force && nowMs - g_lastTelemetryPublishMs < 500) {
    return;
  }

  publishNavigationTelemetry(
    navigationStateName(g_navigationState),
    g_navigationReason,
    g_currentDistanceCm,
    g_distanceTravelledCm,
    (float)countMappedCells() * kMapCellSizeCm * kMapCellSizeCm / 10000.0f,
    g_estimatedHeadingDegrees,
    g_mapOffsetX,
    g_mapOffsetY,
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
  g_mapOffsetX += shiftX;
  g_mapOffsetY += shiftY;
  g_mapPositionX -= shiftX;
  g_mapPositionY -= shiftY;
  x -= shiftX;
  y -= shiftY;
}

static void mapRangeMeasurement(float distanceCm, float headingDegrees)
{
  if (distanceCm <= 0.0f || distanceCm > kSensorMaximumRangeCm) {
    return;
  }

  const float radians = headingDegrees * 0.01745329252f;
  const float directionX = sinf(radians);
  const float directionY = -cosf(radians);
  const float perpendicularX = -directionY;
  const float perpendicularY = directionX;
  const float halfCellCm = kMapCellSizeCm * 0.5f;

  for (float traveledCm = halfCellCm;
       traveledCm < distanceCm;
       traveledCm += halfCellCm) {
    const float beamHalfWidthCm =
        traveledCm * tanf(kUltrasonicBeamHalfAngleRadians);
    for (float lateralCm = -beamHalfWidthCm;
         lateralCm <= beamHalfWidthCm;
         lateralCm += halfCellCm) {
      const float visibleX = g_mapPositionX +
          (directionX * traveledCm + perpendicularX * lateralCm) /
              kMapCellSizeCm;
      const float visibleY = g_mapPositionY +
          (directionY * traveledCm + perpendicularY * lateralCm) /
              kMapCellSizeCm;
      markMapCell((int)lroundf(visibleX), (int)lroundf(visibleY), MAP_FREE);
    }
  }

  if (distanceCm < kSensorMaximumRangeCm) {
    const int obstacleX = (int)lroundf(
        g_mapPositionX + directionX * distanceCm / kMapCellSizeCm);
    const int obstacleY = (int)lroundf(
        g_mapPositionY + directionY * distanceCm / kMapCellSizeCm);
    markMapCell(obstacleX, obstacleY, MAP_OBSTACLE);
  }
}

static void recordEstimatedMovement(float distanceCm)
{
  if (distanceCm == 0.0f) {
    return;
  }

  const float radians = g_estimatedHeadingDegrees * 0.01745329252f;
  g_worldPositionX += sinf(radians) * distanceCm;
  g_worldPositionY += cosf(radians) * distanceCm;
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

static float normalizeHeadingDegrees(float headingDegrees)
{
  while (headingDegrees < 0.0f) {
    headingDegrees += 360.0f;
  }
  while (headingDegrees >= 360.0f) {
    headingDegrees -= 360.0f;
  }
  return headingDegrees;
}

static float signedHeadingErrorDegrees(float targetDegrees, float currentDegrees)
{
  float error = normalizeHeadingDegrees(targetDegrees) -
      normalizeHeadingDegrees(currentDegrees);
  if (error > 180.0f) {
    error -= 360.0f;
  } else if (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

static int chooseSafeTurnDirection()
{
  float leftScore = 0.0f;
  float rightScore = 0.0f;
  int leftCount = 0;
  int rightCount = 0;

  for (int i = 0; i < g_scanSampleCount; ++i) {
    const SensorScanSample& sample = g_scanSamples[i];
    if (!sample.valid) {
      continue;
    }

    const float relativeHeading =
        normalizeHeadingDegrees(sample.headingDeg - g_estimatedHeadingDegrees);
    if (relativeHeading > 180) {
      leftScore += sample.distanceCm;
      ++leftCount;
    } else {
      rightScore += sample.distanceCm;
      ++rightCount;
    }
  }

  if (leftCount > 0) {
    leftScore /= leftCount;
  }
  if (rightCount > 0) {
    rightScore /= rightCount;
  }

  if (leftScore > rightScore) {
    return 1;
  }
  if (rightScore > leftScore) {
    return -1;
  }
  return g_laneChangeDirection;
}

static bool perform360Scan()
{
  if (!g_scanInProgress) {
    g_scanInProgress = true;
    g_scanReturnState = g_navigationState;
    g_scanStartHeadingDegrees = g_estimatedHeadingDegrees;
    g_scanNextStep = 0;
    g_scanDegreesPerCommandDegree = getRotationCalibrationScale();
    g_scanCommandDegrees = 360.0f / g_scanDegreesPerCommandDegree;
    g_scanStepCommandDegrees = kTurnAngleDegrees;
    g_scanStepCount = (uint8_t)ceilf(
        g_scanCommandDegrees / g_scanStepCommandDegrees);
    g_scanSampleCount = 0;
    g_scanLeftScore = 0.0f;
    g_scanRightScore = 0.0f;
    g_scanLeftCount = 0;
    g_scanRightCount = 0;
    memset(g_scanDistancesCm, 0, sizeof(g_scanDistancesCm));
    logNavigationEvent("starting calibrated 360-degree scan");
  }

  for (int i = g_scanNextStep; i < g_scanStepCount; ++i) {
    const float heading = g_estimatedHeadingDegrees;
    float currentDistance = measureAvoidDistanceCm();
    SensorScanSample sample;
    sample.headingDeg = heading;
    sample.distanceCm = currentDistance;
    sample.valid = (currentDistance > 0.0f &&
        currentDistance <= kSensorMaximumRangeCm);

    if (sample.valid) {
      if (g_scanSampleCount < (int)(sizeof(g_scanSamples) /
              sizeof(g_scanSamples[0]))) {
        g_scanSamples[g_scanSampleCount++] = sample;
      }
      const int mapSector = (((int)heading + 15) / 30) % 12;
      g_scanDistancesCm[mapSector] = (uint16_t)min((int)currentDistance, 400);
      mapRangeMeasurement(currentDistance, heading);
      const float relativeHeading =
          normalizeHeadingDegrees(heading - g_scanStartHeadingDegrees);
      if (relativeHeading > 180) {
        g_scanLeftScore += sample.distanceCm;
        ++g_scanLeftCount;
      } else {
        g_scanRightScore += sample.distanceCm;
        ++g_scanRightCount;
      }
    }

    const float completedCommandDegrees = i * g_scanStepCommandDegrees;
    const float remainingCommandDegrees =
        g_scanCommandDegrees - completedCommandDegrees;
    const float turnDegrees =
        remainingCommandDegrees < g_scanStepCommandDegrees
        ? remainingCommandDegrees : g_scanStepCommandDegrees;
    const float estimatedTurnDegrees =
        kTurnAngleDegrees * g_scanDegreesPerCommandDegree;
    if (!performTurnStep(1, turnDegrees,
            estimatedTurnDegrees)) {
      return false;
    }
    g_scanNextStep = (uint8_t)(i + 1);
    if (hasNavigationControlPending() || hasNavigationTargetPending()) {
      return false;
    }
  }

  robot.center();
  delay(150);

  g_scanInProgress = false;
  g_initialScanComplete = (g_scanSampleCount > 0);
  if (g_scanNextStep == g_scanStepCount) {
    logNavigationEvent("calibrated 360-degree scan completed");
  }
  if (!g_initialScanComplete) {
    if (g_navigationState != NAV_INITIAL_SCANNING) {
      return false;
    }
    g_navigationState = NAV_ERROR;
    logNavigationEvent("initial scan produced no usable samples");
    return false;
  }

  if (g_scanLeftCount > 0) {
    g_scanLeftScore /= g_scanLeftCount;
  }
  if (g_scanRightCount > 0) {
    g_scanRightScore /= g_scanRightCount;
  }
  g_laneChangeDirection =
      (g_scanLeftScore >= g_scanRightScore) ? 1 : -1;
  if (g_navigationState == NAV_INITIAL_SCANNING) {
    markMapCell((int)lroundf(g_mapPositionX), (int)lroundf(g_mapPositionY), MAP_ROBOT);
    g_navigationState = NAV_EXPLORING;
    logNavigationEvent("initial scan complete");
  }
  return true;
}

static bool rotateQuarterTurn(int direction)
{
  return rotateByDegrees(direction, 90.0f);
}

static bool rotateByDegrees(int direction, float degrees,
    int executingProgramId)
{
  const float degreesPerCommandDegree = getRotationCalibrationScale();
  const float commandDegrees = degrees / degreesPerCommandDegree;
  if (Servo_PROGRAM >= 1 && Servo_PROGRAM != executingProgramId) {
    return false;
  }
  float remainingDegrees = commandDegrees;
  while (remainingDegrees > 0.001f) {
    const float stepDegrees = remainingDegrees < kTurnAngleDegrees
        ? remainingDegrees : kTurnAngleDegrees;
    if (!performTurnStep(direction, stepDegrees,
            kTurnAngleDegrees * degreesPerCommandDegree,
            executingProgramId)) {
      return false;
    }
    remainingDegrees -= stepDegrees;
    if (hasNavigationControlPending() || hasNavigationTargetPending() ||
        (Servo_PROGRAM >= 1 && Servo_PROGRAM != executingProgramId)) {
      return false;
    }
  }
  return true;
}

static bool performTurnStep(int direction, float degrees,
    float estimatedDegrees, int executingProgramId)
{
  if (degrees <= 0.0f || degrees > kTurnAngleDegrees) {
    return false;
  }
  if (isRobotStopRequested() ||
      (Servo_PROGRAM >= 1 && Servo_PROGRAM != executingProgramId)) {
    return false;
  }
  if (direction > 0) {
    robot.turnleft();
  } else {
    robot.turnright();
  }
  if (isRobotStopRequested()) {
    return false;
  }

  if (estimatedDegrees < 0.0f) {
    estimatedDegrees = degrees;
  }
  g_estimatedHeadingDegrees = normalizeHeadingDegrees(
      g_estimatedHeadingDegrees +
      (direction > 0 ? -estimatedDegrees : estimatedDegrees));
  publishNavigationStatus(true);
  handleClient();
  return !isRobotStopRequested() &&
      (Servo_PROGRAM < 1 || Servo_PROGRAM == executingProgramId);
}

static bool moveForwardSteps(uint8_t steps, bool checkObstacles)
{
  for (uint8_t step = 0; step < steps; ++step) {
    if (checkObstacles) {
      const float distanceCm = measureAvoidDistanceCm();
      if (!g_sensorValid || distanceCm < g_navigationConfig.safeTurnDistanceCm) {
        robot.center();
        g_navigationState = NAV_EXPLORING;
        if (g_sensorValid) {
          mapRangeMeasurement(distanceCm, g_estimatedHeadingDegrees);
        }
        return false;
      }
      mapRangeMeasurement(distanceCm, g_estimatedHeadingDegrees);
    }

    if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
        hasNavigationControlPending() || hasNavigationTargetPending()) {
      return false;
    }
    robot.forward();
    if (isRobotStopRequested()) {
      return false;
    }
    recordEstimatedMovement(kForwardStepDistanceCm);
    g_distanceTravelledCm += kForwardStepDistanceCm;
    g_lastMovementCm = kForwardStepDistanceCm;
    handleClient();
    if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
        hasNavigationControlPending() || hasNavigationTargetPending()) {
      return false;
    }
  }
  return true;
}

static bool changeLane(int direction, bool reverseFirst)
{
  if (reverseFirst) {
    robot.backward();
    if (isRobotStopRequested() || Servo_PROGRAM >= 1) {
      return false;
    }
    recordEstimatedMovement(-kForwardStepDistanceCm);
    g_distanceTravelledCm += kForwardStepDistanceCm;
    g_lastMovementCm = -kForwardStepDistanceCm;
    handleClient();
    if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
        hasNavigationControlPending() || hasNavigationTargetPending()) {
      return false;
    }
  }
  if (!rotateQuarterTurn(direction)) {
    return false;
  }
  if (!moveForwardSteps(kLaneSpacingSteps, true)) {
    if (!isRobotStopRequested() && Servo_PROGRAM < 1 &&
        !hasNavigationControlPending() && !hasNavigationTargetPending()) {
      rotateQuarterTurn(-direction);
    }
    return false;
  }
  if (!rotateQuarterTurn(direction)) {
    return false;
  }

  g_laneChangeDirection = -direction;
  return true;
}

static void stopCoverageAtBoundary()
{
  robot.center();
  g_autonomousEnabled = false;
  g_navigationState = NAV_STOPPED;
  requestRobotStop(PROGRAM_STANDBY);
  logNavigationEvent(
      "coverage complete; no clear adjacent lane remains in the sweep direction");
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

  if (!perform360Scan()) {
    if (hasNavigationControlPending()) {
      return;
    }
    robot.center();
    g_navigationState = NAV_PAUSED;
    logNavigationEvent("recovery scan failed; autonomous navigation paused");
    return;
  }

  if (!changeLane(g_laneChangeDirection, true)) {
    if (!isRobotStopRequested() && Servo_PROGRAM < 1 &&
        !hasNavigationControlPending() && !hasNavigationTargetPending()) {
      stopCoverageAtBoundary();
    }
    return;
  }
  g_blockedCycles = 0;
  g_navigationState = NAV_EXPLORING;
  logNavigationEvent("recovery lane change complete");
}

static void updateAutonomousNavigation()
{
  if (g_navigationState == NAV_TARGETING) {
    updateTargetNavigation();
    return;
  }

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
    perform360Scan();
    return;
  }

  if (g_navigationState == NAV_EXPLORING) {
    float obstacleDistance = measureAvoidDistanceCm();
    if (!g_sensorValid || obstacleDistance <= 0.0f) {
      robot.center();
      if (g_invalidSensorCycles < UINT8_MAX) {
        ++g_invalidSensorCycles;
      }
      if (g_invalidSensorCycles >= kMaxInvalidSensorCycles) {
        const SensorRecoveryResult recoveryResult = recoverUltrasonicSensor();
        if (recoveryResult == SENSOR_RECOVERY_FAILED) {
          g_autonomousEnabled = false;
          g_navigationState = NAV_ERROR;
          logNavigationEvent("sensor recovery failed; autonomous navigation stopped");
        }
      }
      return;
    }
    g_invalidSensorCycles = 0;

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

    if (!moveForwardSteps(1)) {
      return;
    }
    return;
  }

  if (g_navigationState == NAV_AVOIDING_OBSTACLE) {
    robot.center();
    delay(120);
    if (!perform360Scan()) {
      if (Servo_PROGRAM >= 1 || isRobotStopRequested() ||
          hasNavigationControlPending()) {
        return;
      }
      robot.center();
      g_navigationState = NAV_PAUSED;
      logNavigationEvent("obstacle scan failed; autonomous navigation paused");
      return;
    }
    const int turnDirection = g_laneChangeDirection;
    if (!changeLane(turnDirection, false)) {
      if (!isRobotStopRequested() && Servo_PROGRAM < 1 &&
          !hasNavigationControlPending() && !hasNavigationTargetPending()) {
        stopCoverageAtBoundary();
      }
      return;
    }
    g_navigationState = NAV_EXPLORING;
    return;
  }

  if (g_navigationState == NAV_REPOSITIONING) {
    if (!changeLane(g_laneChangeDirection, false)) {
      if (!isRobotStopRequested() && Servo_PROGRAM < 1 &&
          !hasNavigationControlPending() && !hasNavigationTargetPending()) {
        stopCoverageAtBoundary();
      }
      return;
    }
    g_navigationState = NAV_EXPLORING;
    return;
  }

  if (g_navigationState == NAV_RECOVERING) {
    g_navigationState = NAV_EXPLORING;
    return;
  }
}

static void processNavigationTarget()
{
  if (g_navigationState == NAV_STARTUP_DELAY &&
      millis() < g_startupDeadlineMs) {
    return;
  }
  if (Servo_PROGRAM >= 1) {
    return;
  }

  NavigationWaypoint requestedWaypoints[MAX_NAVIGATION_WAYPOINTS];
  uint8_t count = 0;
  const NavigationTargetKind kind =
      takeNavigationTarget(requestedWaypoints, count);
  if (kind == NAV_TARGET_NONE) {
    return;
  }

  robot.center();
  measureAvoidDistanceCm();
  if (!g_sensorValid && recoverUltrasonicSensor() != SENSOR_RECOVERED) {
    g_autonomousEnabled = false;
    g_targetNavigationActive = false;
    g_navigationState = NAV_ERROR;
    logNavigationEvent("target rejected; ultrasonic sensor unavailable");
    return;
  }

  if (kind == NAV_TARGET_HOME) {
    requestedWaypoints[0].xCm = 0.0f;
    requestedWaypoints[0].yCm = 0.0f;
    requestedWaypoints[0].animationId = PROGRAM_NONE;
    count = 1;
  }
  if (count == 0 || count > MAX_NAVIGATION_WAYPOINTS) {
    logNavigationEvent("target rejected; invalid waypoint count");
    return;
  }

  memcpy(g_activeWaypoints, requestedWaypoints,
      count * sizeof(g_activeWaypoints[0]));
  g_activeWaypointCount = count;
  g_activeWaypointIndex = 0;
  g_targetNavigationActive = true;
  g_activeTargetKind = kind;
  g_targetDetourActive = false;
  g_autonomousEnabled = true;
  g_blockedCycles = 0;
  g_recoveryAttempts = 0;
  g_invalidSensorCycles = 0;
  g_navigationState = NAV_TARGETING;
  logNavigationEvent(kind == NAV_TARGET_HOME
      ? "return-home target accepted"
      : kind == NAV_TARGET_ROUTE ? "waypoint route accepted"
      : "go-to target accepted");
}

static void updateTargetNavigation()
{
  if (!g_targetNavigationActive || g_activeWaypointIndex >= g_activeWaypointCount) {
    robot.center();
    g_targetNavigationActive = false;
    g_activeTargetKind = NAV_TARGET_NONE;
    g_autonomousEnabled = false;
    g_navigationState = NAV_PAUSED;
    logNavigationEvent("target navigation stopped; no active waypoint");
    return;
  }
  if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
      hasNavigationControlPending() || hasNavigationTargetPending()) {
    robot.center();
    return;
  }

  const float obstacleDistance = measureAvoidDistanceCm();
  handleClient();
  if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
      hasNavigationControlPending() || hasNavigationTargetPending()) {
    return;
  }
  if (!g_sensorValid || obstacleDistance <= 0.0f) {
    robot.center();
    if (g_invalidSensorCycles < UINT8_MAX) {
      ++g_invalidSensorCycles;
    }
    if (g_invalidSensorCycles >= kMaxInvalidSensorCycles) {
      const SensorRecoveryResult result = recoverUltrasonicSensor();
      if (result == SENSOR_RECOVERY_FAILED) {
        g_autonomousEnabled = false;
        g_targetNavigationActive = false;
        g_navigationState = NAV_ERROR;
        logNavigationEvent("target navigation stopped; sensor recovery failed");
      }
    }
    return;
  }
  g_invalidSensorCycles = 0;
  mapRangeMeasurement(obstacleDistance, g_estimatedHeadingDegrees);

  const NavigationWaypoint& target = g_activeWaypoints[g_activeWaypointIndex];
  const float deltaX = target.xCm - g_worldPositionX;
  const float deltaY = target.yCm - g_worldPositionY;
  const float distanceToTarget = sqrtf(deltaX * deltaX + deltaY * deltaY);
  if (distanceToTarget <= kForwardStepDistanceCm) {
    const int animationId = g_activeWaypoints[g_activeWaypointIndex].animationId;
    g_activeWaypoints[g_activeWaypointIndex].animationId = PROGRAM_NONE;
    if (animationId != PROGRAM_NONE) {
      robot.center();
      runRobotProgram(animationId);
      if (isRobotStopRequested() || Servo_PROGRAM >= 1) {
        return;
      }
      handleClient();
      if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
          hasNavigationControlPending() || hasNavigationTargetPending()) {
        return;
      }
    }
    ++g_activeWaypointIndex;
    if (g_activeWaypointIndex < g_activeWaypointCount) {
      return;
    }

    robot.center();
    g_targetNavigationActive = false;
    if (g_activeTargetKind == NAV_TARGET_POINT && animationId != PROGRAM_NONE) {
      g_autonomousEnabled = true;
      g_navigationState = NAV_INITIAL_SCANNING;
      logNavigationEvent("destination animation complete; resuming autonomous coverage");
    } else {
      g_autonomousEnabled = false;
      g_navigationState = NAV_PAUSED;
      logNavigationEvent("destination reached; navigation paused");
    }
    g_activeTargetKind = NAV_TARGET_NONE;
    return;
  }

  const float targetHeading = g_targetDetourActive
      ? g_targetDetourHeading
      : atan2f(deltaX, deltaY) * 57.2957795f;
  const float headingError =
      signedHeadingErrorDegrees(targetHeading, g_estimatedHeadingDegrees);
  if (fabsf(headingError) > 0.5f &&
      !rotateByDegrees(headingError > 0.0f ? -1 : 1, fabsf(headingError))) {
    return;
  }

  if (obstacleDistance <= g_navigationConfig.safeTurnDistanceCm) {
    robot.center();
    if (g_recoveryAttempts < UINT8_MAX) {
      ++g_recoveryAttempts;
    }
    if (g_recoveryAttempts > g_navigationConfig.maxRecoveryAttempts) {
      g_autonomousEnabled = false;
      g_targetNavigationActive = false;
      g_activeTargetKind = NAV_TARGET_NONE;
      g_activeTargetKind = NAV_TARGET_NONE;
      g_navigationState = NAV_STOPPED;
      requestRobotStop(PROGRAM_STANDBY);
      logNavigationEvent("target blocked; recovery limit reached and robot stopped");
      return;
    }

    if (!perform360Scan()) {
      if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
          hasNavigationControlPending() || hasNavigationTargetPending()) {
        return;
      }
      g_autonomousEnabled = false;
      g_targetNavigationActive = false;
      g_navigationState = NAV_PAUSED;
      logNavigationEvent("target obstacle scan failed; navigation paused");
      return;
    }

    float bestHeadingError = 181.0f;
    float bestHeading = -1.0f;
    for (int i = 0; i < g_scanSampleCount; ++i) {
      const SensorScanSample& sample = g_scanSamples[i];
      if (!sample.valid ||
          sample.distanceCm < g_navigationConfig.safeTurnDistanceCm + 5.0f) {
        continue;
      }
      const float candidateHeading = normalizeHeadingDegrees(sample.headingDeg);
      const float desiredHeading = atan2f(deltaX, deltaY) * 57.2957795f;
      const float error = fabsf(
          signedHeadingErrorDegrees(candidateHeading, desiredHeading));
      if (error < bestHeadingError) {
        bestHeadingError = error;
        bestHeading = candidateHeading;
      }
    }
    if (bestHeading < 0) {
      const int turnDirection = chooseSafeTurnDirection();
      bestHeading = normalizeHeadingDegrees(g_estimatedHeadingDegrees +
          (turnDirection > 0 ? 270.0f : 90.0f));
    }
    g_targetDetourActive = true;
    g_targetDetourHeading = bestHeading;
    const float detourError = signedHeadingErrorDegrees(
        g_targetDetourHeading, g_estimatedHeadingDegrees);
    if (fabsf(detourError) > 0.5f &&
        !rotateByDegrees(detourError > 0.0f ? -1 : 1,
            fabsf(detourError))) {
      return;
    }
    return;
  }

  if (isRobotStopRequested() || Servo_PROGRAM >= 1 ||
      hasNavigationControlPending() || hasNavigationTargetPending()) {
    return;
  }
  robot.forward();
  if (isRobotStopRequested()) {
    return;
  }
  recordEstimatedMovement(kForwardStepDistanceCm);
  g_distanceTravelledCm += kForwardStepDistanceCm;
  g_lastMovementCm = kForwardStepDistanceCm;
  g_targetDetourActive = false;
  g_recoveryAttempts = 0;
  g_blockedCycles = 0;
  handleClient();
}

static void processNavigationControl()
{
  const NavigationControl control = takeNavigationControl();
  if (control == NAV_CONTROL_NONE) {
    return;
  }

  if (Servo_PROGRAM >= 1) {
    logNavigationEvent("navigation command skipped because manual action is pending");
    return;
  }

  if (control == NAV_CONTROL_PAUSE) {
    clearRobotStopRequest();
    robot.center();
    g_autonomousEnabled = false;
    g_navigationState = NAV_PAUSED;
    logNavigationEvent("autonomous navigation paused by user");
    return;
  }

  clearRobotStopRequest();

  if (control == NAV_CONTROL_RESTART) {
    robot.center();
    g_scanInProgress = false;
    g_scanNextStep = 0;
    memset(g_occupancyGrid, MAP_UNKNOWN, sizeof(g_occupancyGrid));
    memset(g_scanDistancesCm, 0, sizeof(g_scanDistancesCm));
    g_mapPositionX = kMapOriginX;
    g_mapPositionY = kMapOriginY;
    g_mapOffsetX = (int)lroundf(g_worldPositionX / kMapCellSizeCm) - kMapOriginX;
    g_mapOffsetY = (int)lroundf(-g_worldPositionY / kMapCellSizeCm) - kMapOriginY;
    g_targetNavigationActive = false;
    g_targetDetourActive = false;
    g_activeWaypointCount = 0;
    g_distanceTravelledCm = 0.0f;
    g_lastMovementCm = 0.0f;
    g_recoveryAttempts = 0;
    g_blockedCycles = 0;
    g_invalidSensorCycles = 0;
    g_initialScanComplete = false;
    g_laneChangeDirection = 1;
    markMapCell(kMapOriginX, kMapOriginY, MAP_ROBOT);
  } else {
    robot.center();
  }

  measureAvoidDistanceCm();
  if (!g_sensorValid &&
      recoverUltrasonicSensor() != SENSOR_RECOVERED) {
    g_autonomousEnabled = false;
    g_navigationState = NAV_ERROR;
    logNavigationEvent("navigation resume failed; ultrasonic sensor unavailable");
    return;
  }

  g_recoveryAttempts = 0;
  g_blockedCycles = 0;
  g_invalidSensorCycles = 0;
  if (control == NAV_CONTROL_RESUME && g_scanInProgress) {
    g_autonomousEnabled = true;
    g_navigationState = g_scanReturnState;
    logNavigationEvent("resuming interrupted scan at its remaining turn steps");
    return;
  }
  if (control == NAV_CONTROL_RESUME && g_targetNavigationActive) {
    g_autonomousEnabled = true;
    g_navigationState = NAV_TARGETING;
    logNavigationEvent("target navigation resumed");
    return;
  }
  g_autonomousEnabled = true;
  g_navigationState = NAV_INITIAL_SCANNING;
  logNavigationEvent(control == NAV_CONTROL_RESTART
      ? "autonomous navigation restarted"
      : "autonomous navigation resumed; rescanning surroundings");
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

  beginWebStorage();
  robot.init();
  robot.Servo_PROGRAM_Zero();
  markMapCell((int)lroundf(g_mapPositionX), (int)lroundf(g_mapPositionY), MAP_ROBOT);

  const bool wifiStarted = webinit();
  enableWebServer();
  g_nextWifiRetryMs = millis() + 10000;
  if (!wifiStarted) {
    logNavigationEvent(
        "Wi-Fi access point failed to start; retrying while navigation initializes");
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
  if (!isWifiApReady() && (int32_t)(millis() - g_nextWifiRetryMs) >= 0) {
    webinit();
    g_nextWifiRetryMs = millis() + 10000;
  }

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
      g_scanInProgress = false;
      g_scanNextStep = 0;
      runRobotProgram(programId);
    }

    DEBUG_PRINTLN(irnum);
    irnum = 0;
  }

  handleClient();
  processNavigationControl();
  processNavigationTarget();

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
    g_scanInProgress = false;
    g_scanNextStep = 0;
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
  g_currentDistanceCm = measuredDistanceCm < kSensorMaximumRangeCm
      ? measuredDistanceCm : kSensorMaximumRangeCm;
  g_sensorValid = (measuredDistanceCm > 0.0f);

  if (g_sensorValid && g_navigationState == NAV_EXPLORING) {
    mapRangeMeasurement(g_currentDistanceCm, g_estimatedHeadingDegrees);
  }
  if (g_sensorValid && g_initialScanComplete) {
    int mapSector = ((int)lroundf(g_estimatedHeadingDegrees + 15.0f) / 30) % 12;
    g_scanDistancesCm[mapSector] =
        (uint16_t)min((int)g_currentDistanceCm, 400);
  }
  return g_currentDistanceCm;
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
      rotateByDegrees(1, 2.0f * kTurnAngleDegrees, PROGRAM_AVOID);
    } else if (distance >= AVOIDDIS) {
      robot.forward();
      if (isRobotStopRequested()) {
        break;
      }
      recordEstimatedMovement(kForwardStepDistanceCm);
      g_distanceTravelledCm += kForwardStepDistanceCm;
      g_lastMovementCm = kForwardStepDistanceCm;
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
      rotateByDegrees(1, kTurnAngleDegrees, programId);
      break;
    case PROGRAM_TURNRIGHT:
      rotateByDegrees(-1, kTurnAngleDegrees, programId);
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

  if (!isRobotStopRequested()) {
    if (programId == PROGRAM_FORWARD || programId == PROGRAM_BACKWARD) {
      const float movementCm = programId == PROGRAM_FORWARD
          ? kForwardStepDistanceCm : -kForwardStepDistanceCm;
      recordEstimatedMovement(movementCm);
      g_distanceTravelledCm += kForwardStepDistanceCm;
      g_lastMovementCm = movementCm;
    }
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
