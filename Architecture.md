# SpiderBot Architecture

## 1. Purpose and scope

SpiderBot is an Arduino-based quadruped robot controller with an ESP8266 Wi-Fi
access point, an embedded web dashboard, manual actions, and sensor-guided
autonomous navigation. The firmware combines three main concerns:

1. **Motion execution** — playback of servo keyframe programs.
2. **Navigation** — obstacle sensing, estimated pose, a local occupancy map,
   autonomous coverage, and waypoint navigation.
3. **Operator control** — a browser dashboard and HTTP API, plus infrared
   remote input.

The design is a single-threaded embedded application. The Arduino `loop()`
dispatches input, services HTTP requests, and advances navigation; motion
programs are synchronous but periodically service the web server so requests
and stop signals can be handled during a gait.

## 2. System context

```mermaid
flowchart LR
    Operator[Operator browser]
    Remote[IR remote]
    ESP[ESP8266 firmware]
    Range[Ultrasonic range sensor]
    Servo[Eight servo actuators]
    EEPROM[(EEPROM emulation)]

    Operator <-->|Wi-Fi AP / HTTP| ESP
    Remote -->|IR receiver| ESP
    Range -->|Trigger / echo| ESP
    ESP -->|Servo PWM| Servo
    ESP <-->|Calibration values| EEPROM
```

The firmware estimates movement from completed gait commands. It has no
inertial measurement unit, wheel encoders, or other independent pose sensor in
the code. Consequently, heading, distance, route progress, and map position
are estimates, not closed-loop measurements of the robot's actual pose.

## 2.1 Wiring schematic

The following is the logical pin wiring specified by the firmware. `GPIOx`
means the ESP8266 GPIO number; board silkscreen labels such as `D1` vary by
ESP8266 module and should be checked against that board's pinout before
connecting wires.

```text
                                +----------------------+
                                |      ESP8266          |
                                |                      |
 Servo 0 signal  <--------------| GPIO14  (D5*)        |
 Servo 1 signal  <--------------| GPIO12  (D6*)        |
 Servo 2 signal  <--------------| GPIO13  (D7*)        |
 Servo 3 signal  <--------------| GPIO15  (D8*)        |
 Servo 4 signal  <--------------| GPIO16  (D0*)        |
 Servo 5 signal  <--------------| GPIO5   (D1*)        |
 Servo 6 signal  <--------------| GPIO4   (D2*)        |
 Servo 7 signal  <--------------| GPIO2   (D4*)        |
                                |                      |
 Ultrasonic TRIG -------------->| GPIO1 / TX           |
 Ultrasonic ECHO <--------------| GPIO3 / RX           |
                                |                      |
 IR receiver OUT -------------->| D3 / GPIO0            |
                                +----------------------+

 External servo supply +  ------> Servo power rails (all 8 servos)
 External servo supply GND ------+-------------------+
                                  |                   |
 ESP8266 GND --------------------+-------------------+--> Sensor GND
                                                      +--> IR receiver GND

 Sensor VCC <--- supply compatible with the sensor module
 IR receiver VCC <--- regulated supply compatible with the receiver
```

`*` The D-labels shown are common ESP8266 NodeMCU-style aliases for those
GPIOs; not every ESP8266 board uses the same silk labels. Use the GPIO numbers
as the firmware-level identifiers.

### Pin and connector table

| Firmware signal | ESP8266 pin | Direction at ESP | Connected device | Notes |
|---|---:|---|---|---|
| Servo 0 signal | GPIO14 | Output | Right-front foot swing | Logical channel 0. |
| Servo 1 signal | GPIO12 | Output | Right-front leg lift | Logical channel 1. |
| Servo 2 signal | GPIO13 | Output | Right-rear leg lift | Logical channel 2. |
| Servo 3 signal | GPIO15 | Output | Right-rear foot swing | Logical channel 3. |
| Servo 4 signal | GPIO16 | Output | Left-front foot swing | Logical channel 4. |
| Servo 5 signal | GPIO5 | Output | Left-front leg lift | Logical channel 5. |
| Servo 6 signal | GPIO4 | Output | Left-rear leg lift | Logical channel 6. |
| Servo 7 signal | GPIO2 | Output | Left-rear foot swing | Logical channel 7. |
| Ultrasonic TRIG | GPIO1 / TX | Output | Ultrasonic sensor TRIG | Pulsed low/high/low for each range measurement. |
| Ultrasonic ECHO | GPIO3 / RX | Input | Ultrasonic sensor ECHO | Read with `pulseIn()` and a 30 ms timeout. |
| IR receiver data | D3 / GPIO0 | Input | IR receiver OUT | Used by the ESP8266 build. ESP32-C3 conditional build uses GPIO10. |
| Ground reference | GND | — | ESP8266, servo supply, sensor, IR receiver | Grounds must share a reference for signal levels. |
| Servo power | External supply | — | Servo V+ rails | Power requirements depend on the installed servos; the firmware does not specify a supply voltage/current. |
| Sensor power | Module-dependent | — | Ultrasonic sensor VCC | Use the voltage required by the specific sensor module. |
| IR receiver power | Module-dependent | — | IR receiver VCC | Use a supply compatible with the receiver and ESP8266 signal levels. |

The code configures servo pulse limits of 400–2400 microseconds. It does not
specify or validate the power supply. Use a supply rated for the servo set and
keep servo motor current off the ESP8266 regulator; join supply ground to
ESP8266 ground. Confirm the ultrasonic module's echo voltage is safe for the
ESP8266 input before wiring it, since module output levels vary.

**UART pin conflict:** GPIO1/TX and GPIO3/RX are assigned to the ultrasonic
sensor. Enabling serial logging or attaching a serial console can therefore
interfere with range sensing unless the wiring or firmware pin assignment is
changed. GPIO0, GPIO2, GPIO15, and other ESP8266 pins also have boot-strapping
constraints; attached peripherals must not force them into invalid boot levels.

## 3. Source-file responsibilities

| File | Responsibility |
|---|---|
| [`SpiderBot.ino`](./SpiderBot.ino) | Firmware entry points, infrared command dispatch, ultrasonic measurement, navigation state machine, target handling, pose/map estimates, and navigation telemetry publication. |
| [`spiderbot_motion.h`](./spiderbot_motion.h) | `SpiderBotMotion` interface for servo setup, full turn gaits, and poses. |
| [`spiderbot_motion.cpp`](./spiderbot_motion.cpp) | Servo initialization and pin mapping, EEPROM servo offsets, keyframe interpolation, forward steering correction, and motion implementations. |
| [`actionarray.h`](./actionarray.h) | Servo poses and gait keyframe matrices, including forward/backward walking, shifts, turns, and animations. |
| [`webserver.h`](./webserver.h) | Shared action IDs, navigation/API structures, telemetry shape, and cross-module function declarations. |
| [`webserver.cpp`](./webserver.cpp) | ESP8266 SoftAP and HTTP server, embedded HTML/CSS/JavaScript dashboard, REST-style APIs, navigation command queues, and persistent calibration values. |

## 4. Runtime architecture

### 4.1 Startup

`setup()` performs the following sequence:

1. Enables optional debug serial output and starts the IR receiver.
2. Configures the ultrasonic trigger and echo pins.
3. Initializes EEPROM-backed calibration storage, attaches the servo channels,
   and moves them to the configured zero pose.
4. Starts the Wi-Fi access point and HTTP server.
5. Takes an ultrasonic reading and retries sensor recovery if necessary.
6. If the AP fails, the main loop retries it every ten seconds. Sensor-valid
   navigation can still start after the five-second `STARTUP_DELAY`, so an AP
   startup failure does not prevent servo movement.

The AP name includes the last two bytes of the ESP's MAC address. The AP
credential is configured in firmware. The dashboard and API are served
directly from the device. The dashboard markup is kept in
[`dashboard_html.h`](./dashboard_html.h) as a compile-time `PROGMEM` string
included by `webserver.cpp`; editing the page requires rebuilding and
reflashing the firmware, but does not require a separate filesystem upload.

### 4.2 Main loop and action dispatch

```mermaid
flowchart TD
    L[loop] --> IR[Read and dispatch IR input]
    IR --> HTTP[Service HTTP client]
    HTTP --> CTRL[Process queued navigation control]
    CTRL --> TARGET[Process queued waypoint / route target]
    TARGET --> STOP[Complete pending stop / pose request]
    STOP --> ACTION[Run requested manual program]
    ACTION --> SERVO[Apply direct servo debug command]
    SERVO --> NAV{Navigation active?}
    NAV -->|Yes| FSM[Advance navigation state machine]
    NAV -->|No| TELEMETRY[Publish throttled telemetry]
    FSM --> TELEMETRY
    TELEMETRY --> L
```

Manual action IDs are shared by the web UI, API, and IR mapping. They cover
standby, forward/backward, left/right shift, left/right turn, lie, hello,
fighting, push-up, sleep, three dances, and Avoid mode. The code also provides
center and zero-pose commands.

Manual actions pause autonomous navigation. The web/API command path places a
requested program in `Servo_PROGRAM`; the main loop runs it through
`runRobotProgram()`. Stop requests are checked by the motion playback loop and
can end a gait before its keyframes finish.

### 4.3 Motion layer

`SpiderBotMotion` owns eight servo objects and maps logical matrix columns to
the configured GPIO channels. `actionarray.h` describes each gait as a
sequence of eight servo targets plus a duration. The motion layer interpolates
from the cached servo positions toward each keyframe, applies per-servo
EEPROM offsets, and services the HTTP client while animating.

The forward gait represents an estimated 8 cm step. When forward steering
correction is non-zero, the motion layer scales the left and right foot-swing
keyframe excursions in opposite directions during the forward gait. Positive
values increase left-side stride and reduce right-side stride to counter left
drift; negative values reverse that bias. The adjustment is open-loop and does
not measure actual heading or drift.

## 5. Autonomous navigation

### 5.1 Navigation states

| State | Role |
|---|---|
| `INITIALIZING` | Initial firmware state. |
| `STARTUP_DELAY` | Wait before enabling autonomous behavior. |
| `INITIAL_SCANNING` | Rotate in calibrated steps, sample range, and build initial map data. |
| `EXPLORING` | Check forward range and walk forward when clear. |
| `AVOIDING_OBSTACLE` | Scan around an obstacle, then shift in the established sweep direction. |
| `REPOSITIONING` | Shift into the next coverage lane. |
| `RECOVERING` | Retry a blocked route or invalid sensor readings. |
| `TARGETING` | Move toward the active waypoint, route point, or home. |
| `PAUSED` | Hold navigation until the operator resumes or restarts. |
| `STOPPED` | Stop when the adjacent sweep lane is blocked or recovery limits are exhausted. |
| `ERROR` | Report a startup, sensor, or navigation failure. |

### 5.2 Coverage and obstacle handling

The default coverage parameters are:

| Parameter | Current value |
|---|---:|
| Forward step distance used for estimates | 8 cm |
| Nominal turn gait angle | 18.70° |
| Safe-turn distance | 30 cm |
| Initial and obstacle scan gait | Full turn gait |
| Lane spacing | 1 forward gait step (8 cm) |
| Blocked cycles before recovery | 5 |
| Maximum recovery attempts | 3 |
| Invalid sensor cycles before recovery | 5 |

After a calibrated 360° scan, the robot follows a serpentine sweep. It walks a
lane until the front range detects the room boundary or another obstacle, turns
and shifts one 8 cm step into the next lane, then reverses direction. It no
longer stops a lane at an arbitrary 12-step limit. If the next lane shift in
the sweep direction is blocked, it restores its original heading and stops
instead of repeatedly turning from a partial pose.

The initial scan compares average left/right ranges to choose a sweep direction;
later obstacle scans preserve that direction to keep the zig-zag consistent.
Initial mapping and obstacle scans use only complete turn gaits, sampling once
per gait. The map heading advances by the calibrated estimate for each complete
gait, including when the requested total rotation ends partway through a gait.
Each valid range maps the measured endpoint as an obstacle and marks the
intervening sonar beam as observed clear, using a nominal 7.5° half-angle.
Unobserved cells remain unknown; a missing echo is treated as an invalid sensor
reading rather than assumed to mean open space. If obstruction persists,
recovery is attempted; after the configured limit, navigation stops.

The firmware stores a 61 × 61 grid at 10 cm per cell. The API crops the map to
the observed bounds with padding and publishes a dynamic square dimension, so
the mini-map grows as exploration reveals more of the room (up to the grid
capacity). The fixed-size backing grid is recentered as the estimated robot
position approaches its edge; cells beyond capacity may eventually leave the
map.

An invalid or timed-out ultrasonic reading is treated as unavailable, not as a
clear path. Recovery retries the sensor while the robot is stationary. If
recovery fails, autonomous motion is disabled.

```mermaid
flowchart TD
    Start[Startup checks] --> Delay[Wait startup delay]
    Delay --> Scan[Initial 360-degree scan]
    Scan --> Valid{Scan has usable samples?}
    Valid -->|No| Error[ERROR / stop autonomous navigation]
    Valid -->|Yes| Explore[EXPLORING]
    Explore --> Range{Valid front range?}
    Range -->|No| RecoverSensor[Stationary sensor recovery]
    RecoverSensor -->|Recovered| Explore
    RecoverSensor -->|Failed| Error
    Range -->|Clear| Step[Forward gait and update estimated map position]
    Step --> Explore
    Range -->|Obstacle| Avoid[Scan and keep sweep direction]
    Avoid --> Shift[Turn and shift one step]
    Shift -->|Shift clear| Explore
    Shift -->|Shift blocked| Stopped[Coverage boundary reached]
    Avoid -->|Repeated blockage| Recover[Recovery scan / lane change]
    Recover -->|Attempts remain| Explore
    Recover -->|Limit reached| Stopped[STOPPED]
```

The 360° scan is resumable when interrupted by a navigation command: its
remaining turn steps are retained in RAM. Each scan step samples the current
heading and distance before turning. Coverage remains estimate-driven: without
an IMU/encoder or a known room boundary, gait drift and irregular obstacles
prevent a mathematical guarantee that every floor point was crossed.

### 5.3 Waypoints, routes, and return home

The dashboard lets an operator select a map point, queue a route of up to 12
points, associate supported animations with points, pause/resume navigation,
or return to the origin. Navigation targets are validated for numeric bounds,
current map bounds, known obstacle cells, and supported animation IDs.

The target controller estimates the heading to the active waypoint from the
estimated world coordinates, turns toward it, and moves in forward gaits.
When a forward sensor reading indicates an obstacle, it scans and selects a
candidate detour heading. This is local scan-guided steering; it is not a
general graph search over the occupancy grid.

At a waypoint, its optional animation is played. A route continues to its next
point; reaching the final destination pauses navigation. A single animated
destination resumes autonomous coverage after the animation. Return-home is
represented as a target at world coordinate `(0, 0)`.

### 5.4 Heading, calibration, and movement accounting

Explicit rotations from manual control and navigation use `rotateByDegrees()`.
It reads one persistent 360° calibration scale, converts the requested
physical angle into a command angle, then splits the result into nominal
18.70° gait units. All turns use complete gait units, and map heading updates
use the full estimated angle of each gait. Scans use the same calibration
scale. Forward steering correction
is a separate persistent value in degrees per meter; it biases left and right
foot-swing stride during the forward gait.

Both calibrations rely on gait-based estimates:

- The turn calibration is based on a measured 360° result and assumes that the
  correction scales linearly to smaller turns and behaves similarly in both
  directions. Its update accounts for the number of complete gait steps used
  by the current calibration, including the overshoot caused by the final
  full-step gait.
- The forward correction assumes a degrees-per-meter effect and applies it
  after each 8 cm forward gait.

Actual heading and travel are not measured by an encoder or IMU. Servo
mechanics, floor friction, gait amplitude scaling, battery level, and payload
can cause actual movement to differ from the displayed estimate.

## 6. Mapping and telemetry

The navigation module maintains a **61 × 61** maximum occupancy grid with a
nominal 10 cm cell size (up to 6.1 m square). It starts with the robot at the
grid origin. Range samples mark the sonar beam as observed clear and the
measured endpoint as an obstacle; completed gaits mark the estimated path and
robot position. The API crops the grid to the observed area plus a small
unknown border, so the mini-map starts compact and gains cells as the robot
discovers a larger room, up to the firmware grid limit.

| Grid value | API symbol | Meaning |
|---|---|---|
| Unknown | `?` | No observation recorded. |
| Free | `.` | Observed clear along a range ray. |
| Obstacle | `#` | Range endpoint marked as an obstacle. |
| Path | `+` | Estimated path traveled. |
| Robot | `R` | Estimated current robot cell. |

As the robot approaches the edge of the map, the grid is shifted to keep the
robot near its center. Map offsets preserve the relationship between grid
cells and world coordinates so dashboard points remain anchored. The home
marker is drawn by the dashboard at world coordinate `(0, 0)`. Map, pose, and
scan state are held in RAM; the code does not persist the map between resets.

Navigation telemetry is copied from the firmware into a shared structure and
published at most every 500 ms, with navigation events and turn updates able
to force an immediate update. The `/api/v1/navigation` response includes:

- State and textual reason.
- Sensor validity and latest range.
- Estimated distance traveled, map-covered area, and heading.
- Map offsets and recovery count.
- Twelve scan-distance sectors.
- Grid dimensions, cell size, and the serialized grid.

The browser polls navigation telemetry every 500 ms and draws the miniature
map, robot heading, home marker, statistics, and transient notices. Browser
speech synthesis announces navigation-state changes when supported and
unmuted; speech is generated by the dashboard browser, not the robot hardware.

## 7. Web dashboard and HTTP API

### 7.1 Dashboard capabilities

The HTML, CSS, and JavaScript are embedded in `webserver.cpp` and served from
flash. The dashboard provides:

- Manual gait and animation controls.
- Navigation resume, pause, restart, and stop controls.
- Live mini-map, fullscreen map, robot heading, home marker, and statistics.
- Right-click and touch long-press map actions for point targets and routes.
- Optional animation selection for a point or route waypoint.
- Servo-offset, forward-steering, and turn calibration controls.
- Browser text-to-speech announcements with a mute toggle.

The debug dashboard is gated by a runtime UI flag toggled by `/debug/open` and
`/debug/close`; this is a UI gate, not an authentication system.

### 7.2 Routes

| Method and path | Purpose |
|---|---|
| `GET /` | Serve dashboard or locked page. |
| `GET /debug/open` | Enable dashboard UI and redirect to `/`. |
| `GET /debug/close` | Disable dashboard UI and redirect to `/`. |
| `GET /controller` | Queue a legacy action or direct servo command. |
| `GET /save` | Store a legacy servo-offset value. |
| `GET /setting1` | Read the eight legacy servo offsets. |
| `GET /api/v1/info` | Device metadata and supported actions. |
| `GET /api/v1/state` | Current/requested action and AP/UI status. |
| `GET /api/v1/navigation` | Navigation state, range, map, and telemetry. |
| `POST /api/v1/navigation/control` | Queue `pause`, `resume`, or `restart`. |
| `POST /api/v1/navigation/target` | Queue a `goto`, `route`, or `home` target. |
| `POST /api/v1/action` | Queue a supported manual robot action. |
| `GET /api/v1/calibration` | Read servo offsets. |
| `POST /api/v1/calibration` | Save all servo offsets. |
| `GET /api/v1/calibration/steering` | Read forward steering correction. |
| `POST /api/v1/calibration/steering` | Save forward steering correction. |
| `GET /api/v1/calibration/rotation` | Read measured and effective turn calibration. |
| `POST /api/v1/calibration/rotation` | Refine and persist turn calibration from a measured 360° result. |
| `POST /api/v1/servo` | Queue a direct servo-position command. |
| `POST /api/v1/stop` | Request stop and choose standby, center, or zero pose. |

API responses use a JSON envelope with `ok`, `code`, and `message`; successful
handlers may include `data`. Invalid requests return an HTTP error status and
an error message.

Navigation controls and targets are queued in shared firmware state and
consumed by the main loop, rather than executing a navigation operation inside
the HTTP handler. Manual actions use the `Servo_PROGRAM` request slot.

## 8. Persistent settings and state

| Setting/state | Storage | Persistence |
|---|---|---|
| Eight servo offsets | EEPROM bytes 0–7 | Persistent across reboot. |
| Forward steering correction | Signed 16-bit value in EEPROM bytes 8 and 16, with version marker byte 18 | Persistent across reboot; legacy byte 8 / marker byte 9 values are still read until a new value is saved. |
| Last measured 360° angle | EEPROM bytes 10–11 plus marker byte 15 | Persistent across reboot. |
| Effective turn-calibration equivalent | EEPROM bytes 12–13 plus marker byte 14 | Persistent across reboot. |
| Occupancy map, pose, scan progress, and active route | RAM globals | Reset on device restart. |
| Voice mute preference | Browser `localStorage` | Local to the operator's browser. |

The calibration code calls `EEPROM.begin(512)` during Wi-Fi initialization.
Servo offsets are applied when servo targets are written. Forward and turn
calibrations are read by the motion/navigation logic when movement is
commanded.

## 9. Hardware and implementation constraints

| Component | Firmware interface |
|---|---|
| Board/network | ESP8266 Wi-Fi SoftAP with `ESP8266WebServer` on port 80. |
| Servos | Eight servo channels attached to GPIO 14, 12, 13, 15, 16, 5, 4, and 2. |
| Ultrasonic sensor | Trigger on GPIO 1 (TX), echo on GPIO 3 (RX), `pulseIn()` timeout 30 ms. |
| IR receiver | `D3` on the ESP8266 build; pin 10 under `ARDUINO_ESP32C3_DEV`. |
| Persistent storage | Arduino EEPROM emulation. |

Debug serial output is disabled by default. The ultrasonic sensor uses the
UART TX/RX pins in the current wiring, so serial diagnostics can conflict with
sensor operation if enabled without changing the wiring or design.

## 10. End-to-end behavior summary

```text
Boot
  -> Start Wi-Fi and HTTP dashboard
  -> Initialize servos and validate range sensor
  -> Wait, then scan surroundings
  -> Autonomous coverage or operator-selected target
  -> Range check before movement
  -> Execute gait, update estimated pose/map, publish telemetry
  -> Scan/reposition/recover when an obstacle or sensor fault is detected
  -> Pause or stop on operator command, destination, or recovery limit
```

The dashboard is the operator interface and telemetry consumer; the ESP8266
firmware remains the authority for action execution, navigation state, sensor
handling, and persistent calibration.
