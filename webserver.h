#ifndef webserver_h
#define webserver_h
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "spiderbot_motion.h"

enum RobotProgramId {
    PROGRAM_NONE = 0,
    PROGRAM_STANDBY = 1,
    PROGRAM_FORWARD = 2,
    PROGRAM_BACKWARD = 3,
    PROGRAM_LEFTSHIFT = 4,
    PROGRAM_RIGHTSHIFT = 5,
    PROGRAM_TURNLEFT = 6,
    PROGRAM_TURNRIGHT = 7,
    PROGRAM_LIE = 8,
    PROGRAM_HELLO = 9,
    PROGRAM_FIGHTING = 10,
    PROGRAM_PUSHUP = 11,
    PROGRAM_SLEEP = 12,
    PROGRAM_DANCE1 = 13,
    PROGRAM_DANCE2 = 14,
    PROGRAM_DANCE3 = 15,
    PROGRAM_AVOID = 21,
    PROGRAM_CENTER = 99,
    PROGRAM_ZERO = 100
};

struct RobotActionInfo {
    int id;
    const char* name;
    const char* label;
};

struct RobotDeviceInfo {
    const char* model;
    const char* firmwareVersion;
    const char* protocolVersion;
};

struct NavigationTelemetry {
    char state[24];
    char reason[96];
    float distanceCm;
    float distanceTravelledCm;
    float areaCoveredM2;
    float headingDegrees;
    int mapOffsetX;
    int mapOffsetY;
    int recoveryAttempts;
    bool sensorValid;
    uint16_t scanDistancesCm[12];
    uint8_t occupancyGrid[31 * 31];
};

enum NavigationControl {
    NAV_CONTROL_NONE,
    NAV_CONTROL_PAUSE,
    NAV_CONTROL_RESUME,
    NAV_CONTROL_RESTART
};

struct NavigationWaypoint {
    float xCm;
    float yCm;
    int16_t animationId;
};

enum NavigationTargetKind {
    NAV_TARGET_NONE,
    NAV_TARGET_POINT,
    NAV_TARGET_ROUTE,
    NAV_TARGET_HOME
};

static const uint8_t MAX_NAVIGATION_WAYPOINTS = 12;

extern int Servo_PROGRAM;
extern int GPIO_ID;
extern String ival;
extern const RobotDeviceInfo kRobotDeviceInfo;

bool webinit();
void enableWebServer();
void handleClient();

void writeKeyValue(int8_t key, int8_t value);
int8_t readKeyValue(int8_t key);
int8_t getForwardSteeringCorrection();
bool setForwardSteeringCorrection(int degreesPerMeter);
uint16_t getObserved360RotationDegrees();
bool setObserved360RotationDegrees(uint16_t observed360);
float getRotationCalibrationScale();
const RobotActionInfo* getRobotActions(size_t& count);
const RobotActionInfo* findRobotActionById(int actionId);
const RobotActionInfo* findRobotActionByName(const String& actionName);
const char* getActionName(int actionId);
const char* getActionLabel(int actionId);
String getDeviceName();
void setCurrentAction(int actionId, bool busy);
void clearCurrentAction();
int getCurrentActionId();
const char* getCurrentActionName();
bool isRobotBusy();
void requestRobotStop(int targetProgram);
void clearRobotStopRequest();
bool isRobotStopRequested();
int getRobotStopTargetProgram();
bool queueNavigationControl(NavigationControl control);
NavigationControl takeNavigationControl();
bool hasNavigationControlPending();
bool queueNavigationTarget(NavigationTargetKind kind,
    const NavigationWaypoint* waypoints, uint8_t count);
NavigationTargetKind takeNavigationTarget(NavigationWaypoint* waypoints,
    uint8_t& count);
bool hasNavigationTargetPending();
void publishNavigationTelemetry(const char* state, const char* reason,
    float distanceCm,
    float distanceTravelledCm, float areaCoveredM2, float headingDegrees,
    int mapOffsetX, int mapOffsetY, int recoveryAttempts,
    bool sensorValid, const uint16_t scanDistancesCm[12],
    const uint8_t occupancyGrid[31 * 31]);

#endif
