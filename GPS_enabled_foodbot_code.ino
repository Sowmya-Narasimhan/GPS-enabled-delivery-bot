#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include "DFRobot_BMM150.h"

// ================= WIFI =================

const char* WIFI_SSID = "FOODBOT";
const char* WIFI_PASS = "12345678";

// ================= MOTOR PINS =================

#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32

// ================= ULTRASONIC =================

#define TRIG_PIN 14
#define ECHO_PIN 27

// ================= GPS =================

#define GPS_RX 16
#define GPS_TX 17

// ================= MAGNETOMETER =================

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);
float offsetX = 3.5;
float offsetY = 17.5;
float offsetZ = -8.0;
float scaleX  = 1.107345;
float scaleY  = 1.146199;
float scaleZ  = 0.816667;
int ARRIVAL_RADIUS=5;

float headingDeg = 0;
//float magDeclination=-12;
// ================= PWM Config =================

#define PWM_FREQ  2000   // 2 kHz (good for motors)
#define PWM_RES   8      // 8-bit → 0–255
#define ENA 4           // left
#define ENB 5           // right

int baseSpeed = 140;
int minSpeed  = 100;
int maxSpeed  = 220;

// ================= OBJECTS =================

WebServer server(80);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ================= NAVIGATION =================

#define MAX_WAYPOINTS 50
#define STOP_RADIUS 2.5
#define SLOW_RADIUS 5.0
#define HEADING_TOLERANCE 10 // degrees

double waypointLat[MAX_WAYPOINTS];
double waypointLng[MAX_WAYPOINTS];
int totalWaypoints = 0;
int currentWaypointIndex = 0;
double currentLat = 0.0;
double currentLng = 0.0;
double targetLat  = 0.0;
double targetLng  = 0.0;
bool gpsFixAvailable = false;
bool targetSet = false;
bool startNavigation = false;
bool waypointReached = false;
bool destinationReached = false;
double segmentHeading = 0;
bool useSegmentHeading = false;
unsigned long gpsWaitStart = 0;
bool waitingForGPS = false;

double desiredHeading = 0;
bool useCompassControl = false;

// ================= ULTRASONIC =================

long duration;
bool obstacleDetected = false;

// ================= MOTOR FUNCTIONS =================

void drive(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  0, 200);
  rightSpeed = constrain(rightSpeed, 0, 200);

  // Direction: forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, rightSpeed);
  ledcWrite(ENB, leftSpeed);
}

void forward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 140);
  ledcWrite(ENB, 140);
}

void slowforward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 100);
  ledcWrite(ENB, 100);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA,  0);
  ledcWrite(ENB, 0);
}

void setupPWM() {
  // Attach PWM directly to pins (ESP32 core 3.x style)

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // Start stopped

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// ================= ULTRASONIC =================

void readUltrasonic() {

  static int hitCount = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
  if (micros() - start > 20000) return;
}

  unsigned long echoStart = micros();

  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - echoStart > 20000) return;
  }

  duration = micros() - echoStart;

  if (duration > 0) {
    float distance = duration * 0.034 / 2;
    if (distance <= 80) hitCount++;
    else hitCount = 0;
  } else {
    hitCount = 0;
  }

obstacleDetected = (hitCount >= 2);
}

// ================= GPS =================

void readGPS() {

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  static double lastLat = 0;
  static double lastLng = 0;

  if (gps.location.isValid()) {

      double newLat = gps.location.lat();
      double newLng = gps.location.lng();

      if (lastLat != 0) {
          double jump = TinyGPSPlus::distanceBetween(
              lastLat, lastLng,
              newLat, newLng
          );

          // Ignore impossible jump (more than 5 meters in 1 cycle)
          if (jump > 5) {
              return;  // discard this GPS reading
          }
      }

      currentLat = newLat;
      currentLng = newLng;

      lastLat = newLat;
      lastLng = newLng;

      gpsFixAvailable = true;
  }
}

// ================= MAGNETOMETER =================

void readMagnetometer() {

  sBmm150MagData_t mag = bmm150.getGeomagneticData();
  float mx = (mag.x - offsetX) * scaleX;
  float my = (mag.y - offsetY) * scaleY;
  float mz = (mag.z - offsetZ) * scaleZ;
  headingDeg = atan2(my, mx) * 180.0 / PI;
  //headingDeg+=magDeclination;
  if (headingDeg < 0) headingDeg += 360.0;

}

// ================= BEARING =================

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {

  double dLon = radians(lon2 - lon1);

  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  return fmod(degrees(atan2(y, x)) + 360, 360);
}
// ================ helper for obstacle =================

enum ObstacleState {
  OBS_NONE,
  OBS_STOP,
  OBS_TURN_RIGHT_45,
  OBS_FORWARD_1S,
  OBS_REALIGN
};

ObstacleState obstacleState = OBS_NONE;
double obsStartHeading = 0;
double obsTargetBearing = 0;
unsigned long obsTimer = 0;

bool turnRightBy45() {

  readMagnetometer();
  double targetHeading = obsStartHeading + 90;

  if (targetHeading >= 360) targetHeading -= 360;
  
  double error = targetHeading - headingDeg;
  
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  if (abs(error) < 3) {   // 3° tolerance
    stopCar();
    return true;
  }

  drive(baseSpeed + 40, 0);    // spin right
  return false;

}

bool realignToTarget() {
  readMagnetometer();
  double targetHeading = obsStartHeading - 90;
  double error = targetHeading - headingDeg;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  drive(0, baseSpeed + 40);     // left

  targetHeading = obsStartHeading;
   error = targetHeading - headingDeg;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

    // Strong turn if large error
  if (abs(error) > 150) {
  if (error > 0)
    drive(0, baseSpeed+40);   // spin left
  else
    drive(baseSpeed+40, 0);   // spin right

  }

  int correction = error * 2.5;
  int rightSpeed = baseSpeed - correction;
  int leftSpeed = baseSpeed + correction;

  //leftSpeed  = constrain(leftSpeed,  minSpeed, maxSpeed);
  //rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  drive(leftSpeed, rightSpeed);
  return true;
}



void handleObstacleAvoidance() {

  switch (obstacleState) {

    case OBS_STOP:
      stopCar();
      readMagnetometer();
      obsStartHeading = headingDeg;
      obsTargetBearing = desiredHeading;
      obstacleState = OBS_TURN_RIGHT_45;
      break;

    case OBS_TURN_RIGHT_45:
      if (turnRightBy45()) {
        obstacleState = OBS_FORWARD_1S;
        obsTimer = millis();
      }
      break;

    case OBS_FORWARD_1S:
      drive(baseSpeed - 20, baseSpeed - 20);
      if (millis() - obsTimer >= 3000) {
        stopCar();
        obstacleState = OBS_REALIGN;
        
      }
      break;

    case OBS_REALIGN:
      realignToTarget();
      obstacleDetected = false;
      obstacleState = OBS_NONE;
      break;

    case OBS_NONE:
      navigateToTarget();
      break;


    default:
      break;
  }
}

// ================= NAVIGATION =================

void loadCurrentWaypoint() {

  if (currentWaypointIndex < totalWaypoints) {

    targetLat = waypointLat[currentWaypointIndex];
    targetLng = waypointLng[currentWaypointIndex];

    targetSet = true;

    Serial.print("Waypoint Loaded: ");
    Serial.print(targetLat, 6);
    Serial.print(", ");
    Serial.println(targetLng, 6);
  }
}

void navigateToTarget() {

  if (!startNavigation || !targetSet || !gpsFixAvailable) {
    stopCar();
    return;
  }

  // ================= WAIT AFTER WAYPOINT =================
  if (waitingForGPS) {
    if (millis() - gpsWaitStart > 1500) {
      waitingForGPS = false;
      useSegmentHeading = true;   // enable compass hold
    }
    return;
  }

  // ================= OBSTACLE =================
  if (obstacleDetected) {
    obstacleState = OBS_STOP;
    stopCar();
    handleObstacleAvoidance();
    return;
  }

  // ================= DISTANCE CHECK =================
  double dist = TinyGPSPlus::distanceBetween(
      currentLat, currentLng,
      targetLat, targetLng
  );

  // ================= ARRIVAL =================
  if (dist <= ARRIVAL_RADIUS) {

    stopCar();

    currentWaypointIndex++;

    if (currentWaypointIndex >= totalWaypoints-1) {
      destinationReached = true;
      startNavigation = false;
      targetSet = false;
      return;
    }

    // Compute heading between previous WP → next WP
    segmentHeading = calculateBearing(
        waypointLat[currentWaypointIndex ],
        waypointLng[currentWaypointIndex ],
        waypointLat[currentWaypointIndex+1],
        waypointLng[currentWaypointIndex+1]
    );

    loadCurrentWaypoint();

    //waitingForGPS = true;     // short stabilization pause
    gpsWaitStart = millis();
    useSegmentHeading = true; 
    

    return;
  }

  // ================= FIRST SEGMENT (NO COMPASS YET) =================
  if (currentWaypointIndex == 0) {
    forward();
    return;
  }

  // ================= COMPASS HOLD MODE =================
  if (useSegmentHeading) {

    readMagnetometer();

    double error = segmentHeading - headingDeg;

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if(abs(error) > HEADING_TOLERANCE){

    float correction = error * 2.0;

    int leftSpeed  = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    // leftSpeed  = constrain(leftSpeed,  minSpeed, maxSpeed);
    // rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

    drive(leftSpeed, rightSpeed);
    return;
    }
    else{
      forward();
    }
  }

  // Fallback
  forward();
}

// ================= WEB HANDLERS =================

void handleRoute() {
  if (server.hasArg("lat") && server.hasArg("lng")) {
    if (totalWaypoints < MAX_WAYPOINTS) {
      waypointLat[totalWaypoints] = server.arg("lat").toDouble();
      waypointLng[totalWaypoints] = server.arg("lng").toDouble();
      totalWaypoints++;
    }
  }
  server.send(200, "text/plain", "WAYPOINT ADDED");
}

void handleStart() {

  useCompassControl = false;
  //obstacleState = OBS_NONE;
  obstacleDetected = false; 

  if (totalWaypoints == 0) {
    server.send(200, "text/plain", "NO WAYPOINTS");
    return;
  }

  currentWaypointIndex = 0;
  loadCurrentWaypoint();

  startNavigation = true;
  destinationReached = false;

  server.send(200, "text/plain", "ROUTE STARTED");
}

void handlePing() {
  server.send(200, "text/plain", "ESP32_ALIVE");
}

void handleGPS() {
  if (!gpsFixAvailable) {
    server.send(200, "text/plain", "gps:not_ready");
    return;
  }
  String msg = "lat:" + String(currentLat, 6) + ",lng:" + String(currentLng, 6);
  server.send(200, "text/plain", msg);
}

void handleStatus() {
  if (destinationReached)
    server.send(200, "text/plain", "status:destination_reached");
  else if (startNavigation)
    server.send(200, "text/plain", "status:navigating");
  else
    server.send(200, "text/plain", "status:idle");
}

void handleClearRoute() {
  totalWaypoints = 0;
  currentWaypointIndex = 0;
  destinationReached = false;
  targetSet = false;
  startNavigation = false;
  useCompassControl = false;
  server.send(200, "text/plain", "ROUTE CLEARED");
}

void handleStop() {
  stopCar();
  startNavigation = false;
  targetSet = false;
  destinationReached = false;
  server.send(200, "text/plain", "STOPPED");
}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  setupPWM();
  stopCar();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Wire.begin(21, 22);

  if (bmm150.begin()) {
  Serial.println("BMM150 INIT FAILED");
  while (1);
  }

  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(100);

  server.on("/route", handleRoute);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/ping", handlePing);
  server.on("/gps", handleGPS);
  server.on("/status", handleStatus);
  server.on("/clear", handleClearRoute);
  server.begin();
}

// ================= LOOP =================

void loop() {

  readGPS();                 // ALWAYS first
  server.handleClient();
  static unsigned long usTimer = 0;
  if (millis() - usTimer > 30) {   // 12.5 Hz
  readUltrasonic();
  usTimer = millis();
  }
  navigateToTarget();

}

