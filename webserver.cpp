#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "webserver.h"
#include "dashboard_html.h"

const char* password = "12345678";

int Servo_PROGRAM = 0;
int GPIO_ID = 100;
String ival;

ESP8266WebServer server(80);

const RobotDeviceInfo kRobotDeviceInfo = {
    "QuadBot-E",
    "1.0.0",
    "v1"
};

static String g_deviceName = "Robot";
static int g_currentActionId = PROGRAM_NONE;
static bool g_robotBusy = false;
static bool g_stopRequested = false;
static int g_stopTargetProgram = PROGRAM_STANDBY;
static volatile NavigationControl g_navigationControl = NAV_CONTROL_NONE;
static NavigationTargetKind g_navigationTargetKind = NAV_TARGET_NONE;
static NavigationWaypoint g_navigationWaypoints[MAX_NAVIGATION_WAYPOINTS];
static uint8_t g_navigationWaypointCount = 0;
static bool g_debugUiEnabled = false;
static bool g_wifiApReady = false;
static NavigationTelemetry g_navigationTelemetry = {
    "INITIALIZING", "Starting up", -1.0f, 0.0f, 0.0f, 0, -15, -15, 0,
    false, {0}, {0}
};

static const RobotActionInfo kRobotActions[] = {
    {PROGRAM_STANDBY, "standby", "待机"},
    {PROGRAM_FORWARD, "forward", "前进"},
    {PROGRAM_BACKWARD, "backward", "后退"},
    {PROGRAM_LEFTSHIFT, "leftshift", "左移"},
    {PROGRAM_RIGHTSHIFT, "rightshift", "右移"},
    {PROGRAM_TURNLEFT, "turnleft", "左转"},
    {PROGRAM_TURNRIGHT, "turnright", "右转"},
    {PROGRAM_LIE, "lie", "躺下"},
    {PROGRAM_HELLO, "hello", "打招呼"},
    {PROGRAM_FIGHTING, "fighting", "战斗"},
    {PROGRAM_PUSHUP, "pushup", "俯卧撑"},
    {PROGRAM_SLEEP, "sleep", "睡眠"},
    {PROGRAM_DANCE1, "dance1", "舞步1"},
    {PROGRAM_DANCE2, "dance2", "舞步2"},
    {PROGRAM_DANCE3, "dance3", "舞步3"},
    {PROGRAM_AVOID, "avoid", "避障"},
    {PROGRAM_CENTER, "center", "中位"},
    {PROGRAM_ZERO, "zero", "归零"}
};

// Dashboard HTML is defined in dashboard_html.h and stored in program flash.


static const char DEBUG_LOCKED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QuadBot Debug Locked</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#0f172a;color:#e5e7eb;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.card{width:min(420px,100%);background:#111827;border:1px solid #243041;border-radius:16px;padding:22px}
h1{margin:0 0 12px;font-size:1.15rem;color:#93c5fd}
p{margin:0 0 10px;line-height:1.5;color:#cbd5e1}
code{background:#0b1220;padding:2px 6px;border-radius:6px}
</style>
</head>
<body>
  <div class="card">
    <h1>Debug UI Disabled</h1>
    <p>The legacy debug web page is currently hidden.</p>
    <p>Open <code>/debug/open</code> when you want to use it, or keep using the <code>/api/v1</code> endpoints.</p>
  </div>
</body>
</html>
)rawliteral";

// ── EEPROM helpers ────────────────────────────────────────────────────────────

void writeKeyValue(int8_t key, int8_t value)
{
    EEPROM.write(key, value);
    EEPROM.commit();
}

// BUG FIX: was always returning 0 instead of the stored value
int8_t readKeyValue(int8_t key)
{
    return (int8_t)EEPROM.read(key);
}

static const int kForwardSteeringValueAddress = 8;
static const int kForwardSteeringMarkerAddress = 9;
static const uint8_t kForwardSteeringMarker = 0xA7;
static const int8_t kDefaultForwardSteeringCorrection = 30;
static const int kRotation360ValueAddress = 12;
static const int kRotationCalibrationMarkerAddress = 14;
static const uint8_t kRotationCalibrationMarker = 0xD3;
static const int kRotationObservedValueAddress = 10;
static const int kRotationObservedMarkerAddress = 15;
static const uint8_t kRotationObservedMarker = 0xD4;
static const uint16_t kDefaultObserved360Degrees = 360;

int8_t getForwardSteeringCorrection()
{
    if (EEPROM.read(kForwardSteeringMarkerAddress) != kForwardSteeringMarker) {
        return kDefaultForwardSteeringCorrection;
    }
    const int8_t correction =
        (int8_t)EEPROM.read(kForwardSteeringValueAddress);
    return correction >= -90 && correction <= 90
        ? correction : kDefaultForwardSteeringCorrection;
}

bool setForwardSteeringCorrection(int degreesPerMeter)
{
    if (degreesPerMeter < -90 || degreesPerMeter > 90) {
        return false;
    }
    EEPROM.write(kForwardSteeringValueAddress, (uint8_t)(int8_t)degreesPerMeter);
    EEPROM.write(kForwardSteeringMarkerAddress, kForwardSteeringMarker);
    return EEPROM.commit();
}

static uint16_t readEepromUint16(int address)
{
    return (uint16_t)EEPROM.read(address) |
        ((uint16_t)EEPROM.read(address + 1) << 8);
}

static void writeEepromUint16(int address, uint16_t value)
{
    EEPROM.write(address, (uint8_t)(value & 0xFF));
    EEPROM.write(address + 1, (uint8_t)(value >> 8));
}

uint16_t getObserved360RotationDegrees()
{
    if (EEPROM.read(kRotationObservedMarkerAddress) !=
        kRotationObservedMarker) {
        return kDefaultObserved360Degrees;
    }
    const uint16_t value = readEepromUint16(kRotationObservedValueAddress);
    return value >= 300 && value <= 420
        ? value : kDefaultObserved360Degrees;
}

uint16_t getRotationCalibrationEquivalentDegrees()
{
    if (EEPROM.read(kRotationCalibrationMarkerAddress) !=
        kRotationCalibrationMarker) {
        return kDefaultObserved360Degrees;
    }
    const uint16_t value = readEepromUint16(kRotation360ValueAddress);
    return value >= 250 && value <= 490
        ? value : kDefaultObserved360Degrees;
}

float getRotationCalibrationScale()
{
    return (float)getRotationCalibrationEquivalentDegrees() / 360.0f;
}

bool setObserved360RotationDegrees(uint16_t observed360)
{
    if (observed360 < 300 || observed360 > 420) {
        return false;
    }
    const uint16_t currentEquivalent =
        getRotationCalibrationEquivalentDegrees();
    const uint16_t updatedEquivalent = (uint16_t)lroundf(
        (float)currentEquivalent * observed360 / 360.0f);
    if (updatedEquivalent < 250 || updatedEquivalent > 490) {
        return false;
    }
    writeEepromUint16(kRotation360ValueAddress, updatedEquivalent);
    EEPROM.write(kRotationCalibrationMarkerAddress,
        kRotationCalibrationMarker);
    writeEepromUint16(kRotationObservedValueAddress, observed360);
    EEPROM.write(kRotationObservedMarkerAddress, kRotationObservedMarker);
    return EEPROM.commit();
}

const RobotActionInfo* getRobotActions(size_t& count)
{
    count = sizeof(kRobotActions) / sizeof(kRobotActions[0]);
    return kRobotActions;
}

const RobotActionInfo* findRobotActionById(int actionId)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actions[i].id == actionId) {
            return &actions[i];
        }
    }

    return nullptr;
}

const RobotActionInfo* findRobotActionByName(const String& actionName)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actionName.equalsIgnoreCase(actions[i].name)) {
            return &actions[i];
        }
    }

    return nullptr;
}

const char* getActionName(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->name : "idle";
}

const char* getActionLabel(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->label : "空闲";
}

String getDeviceName()
{
    return g_deviceName;
}

void setCurrentAction(int actionId, bool busy)
{
    g_currentActionId = actionId;
    g_robotBusy = busy;
}

void clearCurrentAction()
{
    g_currentActionId = PROGRAM_NONE;
    g_robotBusy = false;
}

int getCurrentActionId()
{
    return g_currentActionId;
}

const char* getCurrentActionName()
{
    return getActionName(g_currentActionId);
}

bool isRobotBusy()
{
    return g_robotBusy;
}

void requestRobotStop(int targetProgram)
{
    g_stopRequested = true;
    g_stopTargetProgram = targetProgram;
}

void clearRobotStopRequest()
{
    g_stopRequested = false;
    g_stopTargetProgram = PROGRAM_STANDBY;
}

bool isRobotStopRequested()
{
    return g_stopRequested;
}

int getRobotStopTargetProgram()
{
    return g_stopTargetProgram;
}

bool queueNavigationControl(NavigationControl control)
{
    if (control == NAV_CONTROL_NONE || Servo_PROGRAM >= 1 ||
        g_navigationControl != NAV_CONTROL_NONE ||
        g_navigationTargetKind != NAV_TARGET_NONE) {
        return false;
    }
    g_navigationControl = control;
    return true;
}

NavigationControl takeNavigationControl()
{
    const NavigationControl control = g_navigationControl;
    g_navigationControl = NAV_CONTROL_NONE;
    return control;
}

bool hasNavigationControlPending()
{
    return g_navigationControl != NAV_CONTROL_NONE;
}

bool queueNavigationTarget(NavigationTargetKind kind,
    const NavigationWaypoint* waypoints, uint8_t count)
{
    if (kind == NAV_TARGET_NONE || Servo_PROGRAM >= 1 ||
        g_navigationTargetKind != NAV_TARGET_NONE ||
        g_navigationControl != NAV_CONTROL_NONE ||
        (kind == NAV_TARGET_POINT && count != 1) ||
        (kind == NAV_TARGET_ROUTE && (count == 0 || count > MAX_NAVIGATION_WAYPOINTS)) ||
        (kind == NAV_TARGET_HOME && count != 0) ||
        (count > 0 && waypoints == nullptr)) {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (waypoints[i].xCm != waypoints[i].xCm ||
            waypoints[i].yCm != waypoints[i].yCm ||
            fabsf(waypoints[i].xCm) > 5000.0f ||
            fabsf(waypoints[i].yCm) > 5000.0f) {
            return false;
        }
        const int16_t animationId = waypoints[i].animationId;
        if (animationId != PROGRAM_NONE && animationId != PROGRAM_HELLO &&
            animationId != PROGRAM_PUSHUP && animationId != PROGRAM_LIE &&
            animationId != PROGRAM_FIGHTING && animationId != PROGRAM_SLEEP &&
            animationId != PROGRAM_DANCE1 && animationId != PROGRAM_DANCE2 &&
            animationId != PROGRAM_DANCE3) {
            return false;
        }
        if (kind != NAV_TARGET_HOME) {
            const float localX = waypoints[i].xCm / 10.0f -
                g_navigationTelemetry.mapOffsetX;
            const float localY = -waypoints[i].yCm / 10.0f -
                g_navigationTelemetry.mapOffsetY;
            if (localX < 0.0f || localX >= 31.0f ||
                localY < 0.0f || localY >= 31.0f) {
                return false;
            }
            const int cellX = (int)lroundf(localX);
            const int cellY = (int)lroundf(localY);
            if (cellX >= 31 || cellY >= 31 ||
                g_navigationTelemetry.occupancyGrid[cellY * 31 + cellX] == 2) {
                return false;
            }
        }
        g_navigationWaypoints[i] = waypoints[i];
    }
    g_navigationWaypointCount = count;
    g_navigationTargetKind = kind;
    return true;
}

NavigationTargetKind takeNavigationTarget(NavigationWaypoint* waypoints,
    uint8_t& count)
{
    const NavigationTargetKind kind = g_navigationTargetKind;
    count = g_navigationWaypointCount;
    if (kind != NAV_TARGET_NONE && waypoints != nullptr) {
        for (uint8_t i = 0; i < count; ++i) {
            waypoints[i] = g_navigationWaypoints[i];
        }
    }
    g_navigationWaypointCount = 0;
    g_navigationTargetKind = NAV_TARGET_NONE;
    return kind;
}

bool hasNavigationTargetPending()
{
    return g_navigationTargetKind != NAV_TARGET_NONE;
}

void publishNavigationTelemetry(const char* state, const char* reason,
    float distanceCm, float distanceTravelledCm, float areaCoveredM2,
    float headingDegrees,
    int mapOffsetX, int mapOffsetY, int recoveryAttempts, bool sensorValid,
    const uint16_t scanDistancesCm[12],
    const uint8_t occupancyGrid[31 * 31])
{
    strncpy(g_navigationTelemetry.state, state, sizeof(g_navigationTelemetry.state) - 1);
    g_navigationTelemetry.state[sizeof(g_navigationTelemetry.state) - 1] = '\0';
    strncpy(g_navigationTelemetry.reason, reason,
        sizeof(g_navigationTelemetry.reason) - 1);
    g_navigationTelemetry.reason[sizeof(g_navigationTelemetry.reason) - 1] = '\0';
    g_navigationTelemetry.distanceCm = distanceCm;
    g_navigationTelemetry.distanceTravelledCm = distanceTravelledCm;
    g_navigationTelemetry.areaCoveredM2 = areaCoveredM2;
    g_navigationTelemetry.headingDegrees = headingDegrees;
    g_navigationTelemetry.mapOffsetX = mapOffsetX;
    g_navigationTelemetry.mapOffsetY = mapOffsetY;
    g_navigationTelemetry.recoveryAttempts = recoveryAttempts;
    g_navigationTelemetry.sensorValid = sensorValid;
    memcpy(g_navigationTelemetry.scanDistancesCm, scanDistancesCm,
        sizeof(g_navigationTelemetry.scanDistancesCm));
    memcpy(g_navigationTelemetry.occupancyGrid, occupancyGrid,
        sizeof(g_navigationTelemetry.occupancyGrid));
}

static bool ensureDebugUiEnabled()
{
    if (g_debugUiEnabled) {
        return true;
    }

    server.send(403, "text/plain", "DEBUG_UI_DISABLED");
    return false;
}

static void sendJsonResponse(int statusCode, bool ok, int code, const char* message, const String& data)
{
    String payload = "{\"ok\":";
    payload.reserve(data.length() + 96);
    payload += ok ? "true" : "false";
    payload += ",\"code\":";
    payload += String(code);
    payload += ",\"message\":\"";
    payload += message;
    payload += "\"";
    if (data != "") {
        payload += ",\"data\":";
        payload += data;
    }
    payload += "}";

    server.send(statusCode, "application/json", payload);
}

static void sendJsonError(int statusCode, int code, const char* message)
{
    sendJsonResponse(statusCode, false, code, message, "");
}

static String buildCalibrationJsonArray()
{
    String offsets = "[";
    for (int i = 0; i < ALLSERVOS; i++) {
        if (i > 0) {
            offsets += ",";
        }
        offsets += String((int8_t)EEPROM.read(i));
    }
    offsets += "]";
    return offsets;
}

static String buildActionsJsonArray()
{
    String actions = "[";
    size_t count = 0;
    const RobotActionInfo* actionList = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (i > 0) {
            actions += ",";
        }

        actions += "{\"id\":";
        actions += String(actionList[i].id);
        actions += ",\"name\":\"";
        actions += actionList[i].name;
        actions += "\",\"label\":\"";
        actions += actionList[i].label;
        actions += "\"}";
    }

    actions += "]";
    return actions;
}

static bool extractJsonInt(const String& body, const char* key, int& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int start = colonPos + 1;
    while (start < body.length() && isspace(body[start])) {
        start++;
    }

    int end = start;
    if (end < body.length() && body[end] == '-') {
        end++;
    }

    while (end < body.length() && isdigit(body[end])) {
        end++;
    }

    if (start == end) {
        return false;
    }

    value = body.substring(start, end).toInt();
    return true;
}

static bool extractJsonString(const String& body, const char* key, String& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int firstQuote = body.indexOf('"', colonPos + 1);
    if (firstQuote < 0) {
        return false;
    }

    int secondQuote = body.indexOf('"', firstQuote + 1);
    if (secondQuote < 0) {
        return false;
    }

    value = body.substring(firstQuote + 1, secondQuote);
    return true;
}

static bool resolveActionRequest(int& actionId)
{
    actionId = PROGRAM_NONE;
    String body = server.arg("plain");
    int parsedId = PROGRAM_NONE;
    String actionName;

    if (body != "") {
        if (extractJsonInt(body, "id", parsedId)) {
            actionId = parsedId;
        } else if (extractJsonString(body, "action", actionName)) {
            const RobotActionInfo* action = findRobotActionByName(actionName);
            if (action != nullptr) {
                actionId = action->id;
            }
        }
    }

    if (actionId == PROGRAM_NONE && server.hasArg("id")) {
        actionId = server.arg("id").toInt();
    }

    if (actionId == PROGRAM_NONE && server.hasArg("action")) {
        const RobotActionInfo* action = findRobotActionByName(server.arg("action"));
        if (action != nullptr) {
            actionId = action->id;
        }
    }

    return findRobotActionById(actionId) != nullptr;
}

static bool parseCalibrationOffsets(int8_t offsets[ALLSERVOS])
{
    String body = server.arg("plain");
    if (body == "") {
        return false;
    }

    int keyPos = body.indexOf("\"offsets\"");
    if (keyPos < 0) {
        return false;
    }

    int startBracket = body.indexOf('[', keyPos);
    int endBracket = body.indexOf(']', startBracket);
    if (startBracket < 0 || endBracket < 0 || endBracket <= startBracket) {
        return false;
    }

    String values = body.substring(startBracket + 1, endBracket);
    values.trim();

    for (int i = 0; i < ALLSERVOS; i++) {
        int commaPos = values.indexOf(',');
        String item;

        if (commaPos >= 0) {
            item = values.substring(0, commaPos);
            values = values.substring(commaPos + 1);
        } else {
            item = values;
            values = "";
        }

        item.trim();
        if (item == "") {
            return false;
        }

        int value = item.toInt();
        if (value < -124 || value > 124) {
            return false;
        }

        offsets[i] = (int8_t)value;
    }

    return values.indexOf(',') < 0 && values == "";
}

static bool parseServoCommand(int& servoId, int& servoValue)
{
    servoId = -1;
    servoValue = 0;

    String body = server.arg("plain");
    if (body != "") {
        extractJsonInt(body, "servo", servoId);
        extractJsonInt(body, "value", servoValue);
    }

    if (servoId < 0 && server.hasArg("servo")) {
        servoId = server.arg("servo").toInt();
    }

    if (!server.hasArg("value") && body == "") {
        return false;
    }

    if (body == "" && server.hasArg("value")) {
        servoValue = server.arg("value").toInt();
    }

    if (servoId < 0 || servoId >= ALLSERVOS) {
        return false;
    }

    if (servoValue < PWMRES_Min || servoValue > PWMRES_Max) {
        return false;
    }

    return true;
}

static bool resolveStopMode(int& targetProgram)
{
    targetProgram = PROGRAM_STANDBY;
    String mode;
    String body = server.arg("plain");

    if (body != "") {
        extractJsonString(body, "mode", mode);
    }

    if (mode == "" && server.hasArg("mode")) {
        mode = server.arg("mode");
    }

    if (mode == "" || mode.equalsIgnoreCase("standby")) {
        targetProgram = PROGRAM_STANDBY;
        return true;
    }

    if (mode.equalsIgnoreCase("center")) {
        targetProgram = PROGRAM_CENTER;
        return true;
    }

    if (mode.equalsIgnoreCase("zero")) {
        targetProgram = PROGRAM_ZERO;
        return true;
    }

    return false;
}

// ── WiFi / EEPROM init ────────────────────────────────────────────────────────

bool webinit()
{
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    WiFi.softAPmacAddress(mac);
    String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
    macID.toUpperCase();

    String AP_NameString = "Robot - " + macID;
    g_deviceName = AP_NameString;
    char AP_NameChar[AP_NameString.length() + 1];
    memset(AP_NameChar, 0, AP_NameString.length() + 1);
    for (int i = 0; i < (int)AP_NameString.length(); i++)
        AP_NameChar[i] = AP_NameString.charAt(i);

    WiFi.mode(WIFI_AP);
    g_wifiApReady = WiFi.softAP(AP_NameChar, password);

    EEPROM.begin(512);
    delay(10);
    return g_wifiApReady;
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────

void handleClient()
{
    server.handleClient();
}

// Main page: serve PROGMEM HTML directly (no String allocation)
void handleIndex()
{
    if (!g_debugUiEnabled) {
        server.send_P(200, "text/html", DEBUG_LOCKED_HTML);
        return;
    }

    server.send_P(200, "text/html", INDEX_HTML);
}

// Returns calibration offsets as JSON array [s0, s1, ..., s7]
// Index i = servo i = EEPROM key i
void handleSetting1()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String content = "[";
    for (int i = 0; i < 8; i++) {
        if (i > 0) content += ",";
        content += String((int8_t)EEPROM.read(i));
    }
    content += "]";
    server.send(200, "application/json", content);
}

static void handleApiInfo()
{
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"model\":\"";
    data += kRobotDeviceInfo.model;
    data += "\",\"firmware_version\":\"";
    data += kRobotDeviceInfo.firmwareVersion;
    data += "\",\"protocol_version\":\"";
    data += kRobotDeviceInfo.protocolVersion;
    data += "\",\"wifi_mode\":\"AP\",";
    data += "\"servo_count\":";
    data += String(ALLSERVOS);
    data += ",\"actions\":";
    data += buildActionsJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiState()
{
    int requestedActionId = Servo_PROGRAM;
    bool avoidEnabled = (getCurrentActionId() == PROGRAM_AVOID) || (requestedActionId == PROGRAM_AVOID);
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"current_action_id\":";
    data += String(getCurrentActionId());
    data += ",\"current_action\":\"";
    data += getCurrentActionName();
    data += "\",\"current_action_label\":\"";
    data += getActionLabel(getCurrentActionId());
    data += "\",\"requested_action_id\":";
    data += String(requestedActionId);
    data += ",\"requested_action\":\"";
    data += getActionName(requestedActionId);
    data += "\",\"busy\":";
    data += isRobotBusy() ? "true" : "false";
    data += ",\"avoid_enabled\":";
    data += avoidEnabled ? "true" : "false";
    data += ",\"stop_requested\":";
    data += isRobotStopRequested() ? "true" : "false";
    data += ",\"stop_target\":\"";
    data += getActionName(getRobotStopTargetProgram());
    data += "\",\"debug_ui_enabled\":";
    data += g_debugUiEnabled ? "true" : "false";
    data += ",\"wifi_ap_ready\":";
    data += g_wifiApReady ? "true" : "false";
    data += ",\"wifi_mode\":\"AP\"}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiNavigation()
{
    String data;
    data.reserve(1600);
    data = "{";
    data += "\"state\":\"";
    data += g_navigationTelemetry.state;
    data += "\",\"reason\":\"";
    data += g_navigationTelemetry.reason;
    data += "\",\"sensor_valid\":";
    data += g_navigationTelemetry.sensorValid ? "true" : "false";
    data += ",\"distance_cm\":";
    data += String(g_navigationTelemetry.distanceCm, 1);
    data += ",\"distance_travelled_cm\":";
    data += String(g_navigationTelemetry.distanceTravelledCm, 1);
    data += ",\"area_covered_m2\":";
    data += String(g_navigationTelemetry.areaCoveredM2, 2);
    data += ",\"heading_deg\":";
    data += String(g_navigationTelemetry.headingDegrees, 2);
    data += ",\"map_offset_x\":";
    data += String(g_navigationTelemetry.mapOffsetX);
    data += ",\"map_offset_y\":";
    data += String(g_navigationTelemetry.mapOffsetY);
    data += ",\"recovery_attempts\":";
    data += String(g_navigationTelemetry.recoveryAttempts);
    data += ",\"scan_cm\":[";
    for (int i = 0; i < 12; ++i) {
        if (i > 0) {
            data += ",";
        }
        data += String(g_navigationTelemetry.scanDistancesCm[i]);
    }
    data += "],\"map_width\":31,\"map_height\":31,\"cell_size_cm\":10,\"grid\":\"";
    static const char kMapCellSymbols[] = {'?', '.', '#', '+', 'R'};
    for (size_t i = 0; i < sizeof(g_navigationTelemetry.occupancyGrid); ++i) {
        const uint8_t cell = g_navigationTelemetry.occupancyGrid[i];
        data += cell < sizeof(kMapCellSymbols) ? kMapCellSymbols[cell] : '?';
    }
    data += "\"}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiNavigationControl()
{
    if (!server.hasArg("command")) {
        sendJsonError(400, 4010, "missing navigation command");
        return;
    }

    const String command = server.arg("command");
    NavigationControl control = NAV_CONTROL_NONE;
    if (command == "resume") {
        control = NAV_CONTROL_RESUME;
    } else if (command == "pause") {
        control = NAV_CONTROL_PAUSE;
    } else if (command == "restart") {
        control = NAV_CONTROL_RESTART;
    } else {
        sendJsonError(400, 4011, "invalid navigation command");
        return;
    }

    if (!queueNavigationControl(control)) {
        sendJsonError(409, 4012,
            "navigation command unavailable during a manual action or another pending command");
        return;
    }

    sendJsonResponse(202, true, 0, "navigation command queued", "");
}

static bool parseNavigationPoints(const String& encoded, NavigationWaypoint* points,
    uint8_t& count)
{
    count = 0;
    const char* cursor = encoded.c_str();
    const char* const end = cursor + encoded.length();
    while (cursor < end) {
        if (count >= MAX_NAVIGATION_WAYPOINTS) {
            return false;
        }
        char* parsed = nullptr;
        const float x = strtof(cursor, &parsed);
        if (parsed == cursor || parsed >= end || *parsed != ',' ||
            x != x || fabsf(x) > 5000.0f) {
            return false;
        }
        cursor = parsed + 1;
        const float y = strtof(cursor, &parsed);
        if (parsed == cursor || y != y ||
            fabsf(x) > 5000.0f || fabsf(y) > 5000.0f) {
            return false;
        }
        int16_t animationId = PROGRAM_NONE;
        cursor = parsed;
        if (cursor < end && *cursor == ',') {
            cursor++;
            char* animationEnd = nullptr;
            const long parsedAnimation = strtol(cursor, &animationEnd, 10);
            if (animationEnd == cursor || parsedAnimation < -32768L ||
                parsedAnimation > 32767L) {
                return false;
            }
            animationId = (int16_t)parsedAnimation;
            cursor = animationEnd;
        }
        points[count].xCm = x;
        points[count].yCm = y;
        points[count].animationId = animationId;
        ++count;
        if (cursor == end) {
            break;
        }
        if (*cursor != ';') {
            return false;
        }
        ++cursor;
        if (cursor == end) {
            return false;
        }
    }
    return count > 0;
}

static void handleApiNavigationTarget()
{
    if (!server.hasArg("command")) {
        sendJsonError(400, 4020, "missing navigation target command");
        return;
    }

    const String command = server.arg("command");
    NavigationTargetKind kind = NAV_TARGET_NONE;
    NavigationWaypoint points[MAX_NAVIGATION_WAYPOINTS];
    uint8_t count = 0;
    if (command == "goto") {
        kind = NAV_TARGET_POINT;
    } else if (command == "route") {
        kind = NAV_TARGET_ROUTE;
    } else if (command == "home") {
        kind = NAV_TARGET_HOME;
    } else {
        sendJsonError(400, 4021, "invalid navigation target command");
        return;
    }

    if (kind == NAV_TARGET_POINT || kind == NAV_TARGET_ROUTE) {
        if (!server.hasArg("points") ||
            !parseNavigationPoints(server.arg("points"), points, count) ||
            (kind == NAV_TARGET_POINT && count != 1)) {
            sendJsonError(400, 4022, "invalid navigation target coordinates");
            return;
        }
    }

    if (!queueNavigationTarget(kind, points, count)) {
        sendJsonError(409, 4023,
            "target unavailable, outside the current map, or inside a known obstacle");
        return;
    }
    sendJsonResponse(202, true, 0, "navigation target queued", "");
}

static void handleApiAction()
{
    int actionId = PROGRAM_NONE;
    if (!resolveActionRequest(actionId)) {
        sendJsonError(400, 4001, "invalid action");
        return;
    }

    Servo_PROGRAM = actionId;
    setCurrentAction(actionId, false);

    String data = "{";
    data += "\"id\":";
    data += String(actionId);
    data += ",\"action\":\"";
    data += getActionName(actionId);
    data += "\",\"label\":\"";
    data += getActionLabel(actionId);
    data += "\"}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiCalibrationGet()
{
    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiCalibrationPost()
{
    int8_t offsets[ALLSERVOS];
    if (!parseCalibrationOffsets(offsets)) {
        sendJsonError(400, 4002, "invalid calibration payload");
        return;
    }

    for (int i = 0; i < ALLSERVOS; i++) {
        writeKeyValue(i, offsets[i]);
    }

    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiSteeringCalibrationGet()
{
    String data = "{\"degrees_per_meter\":";
    data += String(getForwardSteeringCorrection());
    data += "}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static bool isJsonIntegerProperty(const String& body, const char* property)
{
    String token = "\"";
    token += property;
    token += "\"";
    const int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }
    const int colonPos = body.indexOf(':', keyPos);
    if (colonPos < 0) {
        return false;
    }
    int cursor = colonPos + 1;
    while (cursor < body.length() && isspace(body[cursor])) {
        ++cursor;
    }
    if (cursor < body.length() && body[cursor] == '-') {
        ++cursor;
    }
    const int digitStart = cursor;
    while (cursor < body.length() && isdigit(body[cursor])) {
        ++cursor;
    }
    if (cursor == digitStart) {
        return false;
    }
    while (cursor < body.length() && isspace(body[cursor])) {
        ++cursor;
    }
    return cursor < body.length() &&
        (body[cursor] == ',' || body[cursor] == '}');
}

static bool parseSteeringCalibrationValue(int& degreesPerMeter)
{
    const String body = server.arg("plain");
    return isJsonIntegerProperty(body, "degrees_per_meter") &&
        extractJsonInt(body, "degrees_per_meter", degreesPerMeter);
}

static void handleApiSteeringCalibrationPost()
{
    int degreesPerMeter = 0;
    if (!parseSteeringCalibrationValue(degreesPerMeter) ||
        degreesPerMeter < -90 || degreesPerMeter > 90) {
        sendJsonError(400, 4005,
            "degrees_per_meter must be a whole number from -90 to 90");
        return;
    }
    if (!setForwardSteeringCorrection(degreesPerMeter)) {
        sendJsonError(500, 5005, "could not persist steering calibration");
        return;
    }

    String data = "{\"degrees_per_meter\":";
    data += String(getForwardSteeringCorrection());
    data += "}";
    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiRotationCalibrationGet()
{
    String data = "{\"observed_360_degrees\":";
    data += String(getObserved360RotationDegrees());
    data += ",\"calibration_equivalent_degrees\":";
    data += String(getRotationCalibrationEquivalentDegrees());
    data += "}";
    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiRotationCalibrationPost()
{
    const String body = server.arg("plain");
    int observed360 = 0;
    if (!isJsonIntegerProperty(body, "observed_360_degrees") ||
        !extractJsonInt(body, "observed_360_degrees", observed360) ||
        observed360 < 300 || observed360 > 420) {
        sendJsonError(400, 4006,
            "observed_360_degrees must be a whole number from 300 to 420");
        return;
    }
    if (!setObserved360RotationDegrees((uint16_t)observed360)) {
        sendJsonError(500, 5006, "could not persist turn calibration");
        return;
    }

    String data = "{\"observed_360_degrees\":";
    data += String(getObserved360RotationDegrees());
    data += ",\"calibration_equivalent_degrees\":";
    data += String(getRotationCalibrationEquivalentDegrees());
    data += "}";
    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiServo()
{
    int servoId = -1;
    int servoValue = 0;
    if (!parseServoCommand(servoId, servoValue)) {
        sendJsonError(400, 4003, "invalid servo payload");
        return;
    }

    GPIO_ID = servoId;
    ival = String(servoValue);

    String data = "{";
    data += "\"servo\":";
    data += String(servoId);
    data += ",\"value\":";
    data += String(servoValue);
    data += "}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiStop()
{
    int targetProgram = PROGRAM_STANDBY;
    if (!resolveStopMode(targetProgram)) {
        sendJsonError(400, 4004, "invalid stop mode");
        return;
    }

    Servo_PROGRAM = PROGRAM_NONE;
    requestRobotStop(targetProgram);

    String data = "{";
    data += "\"mode\":\"";
    data += getActionName(targetProgram);
    data += "\",\"target_program\":";
    data += String(targetProgram);
    data += "}";

    sendJsonResponse(200, true, 0, "stop requested", data);
}

static void handleDebugUiOpen()
{
    g_debugUiEnabled = true;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

static void handleDebugUiClose()
{
    g_debugUiEnabled = false;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

// Save calibration offset: key=servo index (0-7), value=offset (-124~124)
// key=100 resets all offsets to 0
void handleSave()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String key   = server.arg("key");
    String value = server.arg("value");

    const int keyInt = key.toInt();
    const int valueInt = value.toInt();

    delay(50);

    if (keyInt == 100) {
        for (int i = 0; i < 8; i++) writeKeyValue(i, 0);
    } else if (keyInt >= 0 && keyInt < ALLSERVOS &&
        valueInt >= -124 && valueInt <= 124) {
        writeKeyValue((int8_t)keyInt, (int8_t)valueInt);
    } else {
        server.send(400, "text/plain", "Invalid servo calibration");
        return;
    }

    delay(10);
    server.send(200, "text/plain", "OK");
}

// Robot motion control: pm=program number, or servo=index&value=angle
void handleController()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String pm    = server.arg("pm");
    String servo = server.arg("servo");

    if (pm != "") {
        int requestedProgram = pm.toInt();
        if (findRobotActionById(requestedProgram) == nullptr) {
            server.send(400, "text/plain", "INVALID_ACTION");
            return;
        }
        Servo_PROGRAM = requestedProgram;
    }

    if (servo != "") {
        int requestedServo = servo.toInt();
        String value = server.arg("value");

        if (requestedServo < 0 || requestedServo >= ALLSERVOS || value == "") {
            server.send(400, "text/plain", "INVALID_SERVO");
            return;
        }

        GPIO_ID = requestedServo;
        ival    = value;
    }

    server.send(200, "text/plain", "OK");
}

// ── Route registration ────────────────────────────────────────────────────────

void enableWebServer()
{
    server.on("/",           HTTP_GET, handleIndex);
    server.on("/debug/open", HTTP_GET, handleDebugUiOpen);
    server.on("/debug/close", HTTP_GET, handleDebugUiClose);
    server.on("/controller", HTTP_GET, handleController);
    server.on("/save",       HTTP_GET, handleSave);
    server.on("/setting1",   HTTP_GET, handleSetting1);
    server.on("/api/v1/info",        HTTP_GET, handleApiInfo);
    server.on("/api/v1/state",       HTTP_GET, handleApiState);
    server.on("/api/v1/navigation", HTTP_GET, handleApiNavigation);
    server.on("/api/v1/navigation/control", HTTP_POST, handleApiNavigationControl);
    server.on("/api/v1/navigation/target", HTTP_POST, handleApiNavigationTarget);
    server.on("/api/v1/action",      HTTP_POST, handleApiAction);
    server.on("/api/v1/calibration", HTTP_GET, handleApiCalibrationGet);
    server.on("/api/v1/calibration", HTTP_POST, handleApiCalibrationPost);
    server.on("/api/v1/calibration/steering", HTTP_GET,
        handleApiSteeringCalibrationGet);
    server.on("/api/v1/calibration/steering", HTTP_POST,
        handleApiSteeringCalibrationPost);
    server.on("/api/v1/calibration/rotation", HTTP_GET,
        handleApiRotationCalibrationGet);
    server.on("/api/v1/calibration/rotation", HTTP_POST,
        handleApiRotationCalibrationPost);
    server.on("/api/v1/servo",       HTTP_POST, handleApiServo);
    server.on("/api/v1/stop",        HTTP_POST, handleApiStop);

    server.begin();
}
