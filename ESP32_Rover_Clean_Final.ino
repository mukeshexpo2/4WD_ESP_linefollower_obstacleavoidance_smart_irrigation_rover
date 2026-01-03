/*****************************************************
 * 🚗 ESP32 Smart 4WD Rover – CLEAN FINAL VERSION
 * Features:
 * - Line Recovery System (4-phase zigzag search)
 * - Non-blocking mode switching during watering/scanning
 * - WiFi LED status indicator (blink → solid)
 * - Obstacle avoidance with ultrasonic sensor
 * - VL53L0X ToF sensor for target detection
 * - Moisture-based automatic watering
 *****************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

extern "C" {
  #include "driver/ledc.h"
}

/* ================= WiFi ================= */
const char* ssid = "Realme12Pro";
const char* password = "M@123456";

/* ================= Motor Pins ================= */
#define ENA 13
#define IN1 19
#define IN2 14
#define ENB 25
#define IN3 27
#define IN4 26

/* ================= Sensors ================= */
#define SENSOR_LEFT   33
#define SENSOR_CENTER 32
#define SENSOR_RIGHT  35
#define MOISTURE_PIN  34
#define RELAY_PIN     5
#define WIFI_LED      2

/* ================= Ultrasonic Sensor ================= */
#define TRIG_PIN 18
#define ECHO_PIN 17

/* ================= Servo Pins ================= */
#define SERVO_X 4
#define SERVO_Y 23

/* ================= VL53L0X ================= */
Adafruit_VL53L0X lox;
VL53L0X_RangingMeasurementData_t measure;

/* ================= PWM Motors ================= */
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_RES LEDC_TIMER_8_BIT
#define PWM_FREQ 5000
#define PWM_CH_LEFT  LEDC_CHANNEL_0
#define PWM_CH_RIGHT LEDC_CHANNEL_1

/* ================= Servo PWM ================= */
#define SERVO_FREQ 50
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_RES LEDC_TIMER_16_BIT
#define SERVO_CH_X LEDC_CHANNEL_2
#define SERVO_CH_Y LEDC_CHANNEL_3
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

/* ================= Globals ================= */
bool autoMode = false;
int baseSpeed = 200;
int servoXAngle = 90;
int servoYAngle = 90;

/* ================= Line Recovery Globals ================= */
enum Direction { DIR_FORWARD, DIR_LEFT, DIR_RIGHT };
Direction lastDirection = DIR_FORWARD;
unsigned long lostLineTime = 0;
bool isSearching = false;

WebServer server(80);

/* =====================================================
   ULTRASONIC SENSOR
   ===================================================== */
long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if(duration == 0) return 999; // No echo received

  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

/* =====================================================
   SERVO HELPERS
   ===================================================== */
uint32_t angleToDuty(int angle) {
  int us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return (us * ((1 << 16) - 1)) / (1000000 / SERVO_FREQ);
}

void moveServo(int channel, int angle) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, angleToDuty(angle));
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel);
}

void moveServoSmooth(int channel, int currentAngle, int targetAngle, int stepDelay) {
  if(currentAngle < targetAngle) {
    for(int angle = currentAngle; angle <= targetAngle; angle++) {
      moveServo(channel, angle);
      delay(stepDelay);
    }
  } else {
    for(int angle = currentAngle; angle >= targetAngle; angle--) {
      moveServo(channel, angle);
      delay(stepDelay);
    }
  }
}

/* =====================================================
   MOTOR CONTROL
   ===================================================== */
void setupMotorPWM() {
  ledc_timer_config_t t = {
    PWM_MODE, PWM_RES, PWM_TIMER, PWM_FREQ, LEDC_AUTO_CLK
  };
  ledc_timer_config(&t);

  ledc_channel_config_t chL = {ENA, PWM_MODE, PWM_CH_LEFT, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0};
  ledc_channel_config_t chR = {ENB, PWM_MODE, PWM_CH_RIGHT, LEDC_INTR_DISABLE, PWM_TIMER, 0, 0};
  ledc_channel_config(&chL);
  ledc_channel_config(&chR);
}

void setMotorSpeed(int l, int r) {
  ledc_set_duty(PWM_MODE, PWM_CH_LEFT, constrain(l,0,255));
  ledc_update_duty(PWM_MODE, PWM_CH_LEFT);
  ledc_set_duty(PWM_MODE, PWM_CH_RIGHT, constrain(r,0,255));
  ledc_update_duty(PWM_MODE, PWM_CH_RIGHT);
}

void moveForward(int s){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  setMotorSpeed(s,s);
}
void moveBackward(int s){
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  setMotorSpeed(s,s);
}
void turnLeft(int s){
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  setMotorSpeed(s/2,s);
}
void turnRight(int s){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  setMotorSpeed(s,s/2);
}
void stopMotors(){
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  setMotorSpeed(0,0);
}

/* =====================================================
   VL53L0X DISTANCE READ
   ===================================================== */
int getDistanceCM(){
  lox.rangingTest(&measure,false);
  if(measure.RangeStatus==0) return measure.RangeMilliMeter/10;
  return -1;
}

/* =====================================================
   SCANNING & WATERING SEQUENCE (NON-BLOCKING MODE SWITCH)
   ===================================================== */
void scanAndWater() {
  Serial.println("🔍 Scanning mode activated - All IR on black");
  stopMotors();

  // Scan Servo X from 90 → 0 → 180
  for(int angle = 90; angle >= 0; angle--) {
    // CHECK MODE DURING SCAN - ALLOWS MODE SWITCH
    server.handleClient();
    if(!autoMode) {
      Serial.println("⚠️ Manual mode requested - Aborting scan");
      resetServosAndContinue();
      return;
    }

    moveServo(SERVO_CH_X, angle);
    servoXAngle = angle;
    delay(20);

    int dist = getDistanceCM();
    if(dist > 0 && dist < 20) {
      Serial.print("🎯 Target found at angle ");
      Serial.print(angle);
      Serial.print("° - Distance: ");
      Serial.print(dist);
      Serial.println(" cm");

      performWatering();
      return;
    }
  }

  // Continue scanning 0 → 180
  for(int angle = 0; angle <= 180; angle++) {
    // CHECK MODE DURING SCAN - ALLOWS MODE SWITCH
    server.handleClient();
    if(!autoMode) {
      Serial.println("⚠️ Manual mode requested - Aborting scan");
      resetServosAndContinue();
      return;
    }

    moveServo(SERVO_CH_X, angle);
    servoXAngle = angle;
    delay(20);

    int dist = getDistanceCM();
    if(dist > 0 && dist < 20) {
      Serial.print("🎯 Target found at angle ");
      Serial.print(angle);
      Serial.print("° - Distance: ");
      Serial.print(dist);
      Serial.println(" cm");

      performWatering();
      return;
    }
  }

  // No target found, reset and continue
  Serial.println("❌ No target found, resetting...");
  resetServosAndContinue();
}

void performWatering() {
  // Move Y servo to 180 (WITH MODE CHECK)
  Serial.println("📐 Moving Y servo to 180°");

  int currentAngle = servoYAngle;
  int targetAngle = 180;
  if(currentAngle < targetAngle) {
    for(int angle = currentAngle; angle <= targetAngle; angle++) {
      server.handleClient();
      if(!autoMode) {
        Serial.println("⚠️ Manual mode - Stopping servo movement");
        servoYAngle = angle;
        resetServosAndContinue();
        return;
      }
      moveServo(SERVO_CH_Y, angle);
      delay(15);
    }
  }
  servoYAngle = 180;

  // Read moisture sensor
  int moistureValue = analogRead(MOISTURE_PIN);
  Serial.print("💧 Moisture reading: ");
  Serial.println(moistureValue);

  // Threshold: Lower value = more moisture (adjust as needed)
  if(moistureValue < 2000) {
    Serial.println("✅ Moisture detected - Skipping watering");
  } else {
    Serial.println("💦 Dry soil detected - Watering for 5 seconds");
    digitalWrite(RELAY_PIN, HIGH);  // Turn ON relay

    // BREAKABLE 5 SECOND DELAY (50 x 100ms)
    for(int i = 0; i < 50; i++) {
      server.handleClient();
      if(!autoMode) {
        Serial.println("⚠️ Manual mode - Stopping watering");
        digitalWrite(RELAY_PIN, LOW);
        resetServosAndContinue();
        return;
      }
      delay(100);
    }

    digitalWrite(RELAY_PIN, LOW); // Turn OFF relay
    Serial.println("✅ Watering complete");
  }

  resetServosAndContinue();
}

void resetServosAndContinue() {
  // Reset Y servo to 90
  Serial.println("🔄 Resetting servos to home position");

  int currentY = servoYAngle;
  if(currentY < 90) {
    for(int angle = currentY; angle <= 90; angle++) {
      server.handleClient();
      if(!autoMode) {
        servoYAngle = angle;
        return;
      }
      moveServo(SERVO_CH_Y, angle);
      delay(15);
    }
  } else {
    for(int angle = currentY; angle >= 90; angle--) {
      server.handleClient();
      if(!autoMode) {
        servoYAngle = angle;
        return;
      }
      moveServo(SERVO_CH_Y, angle);
      delay(15);
    }
  }
  servoYAngle = 90;

  // Reset X servo to 90
  int currentX = servoXAngle;
  if(currentX < 90) {
    for(int angle = currentX; angle <= 90; angle++) {
      server.handleClient();
      if(!autoMode) {
        servoXAngle = angle;
        return;
      }
      moveServo(SERVO_CH_X, angle);
      delay(15);
    }
  } else {
    for(int angle = currentX; angle >= 90; angle--) {
      server.handleClient();
      if(!autoMode) {
        servoXAngle = angle;
        return;
      }
      moveServo(SERVO_CH_X, angle);
      delay(15);
    }
  }
  servoXAngle = 90;

  // Move forward briefly (only if still in auto mode)
  if(autoMode) {
    Serial.println("⏩ Moving forward");
    moveForward(baseSpeed);

    for(int i = 0; i < 5; i++) {
      server.handleClient();
      if(!autoMode) {
        stopMotors();
        return;
      }
      delay(100);
    }

    stopMotors();
    delay(200);
    Serial.println("🔁 Resuming line following");
  }
}

/* =====================================================
   LINE RECOVERY SEARCH PATTERN
   ===================================================== */
void performLineSearch() {
  unsigned long searchDuration = millis() - lostLineTime;

  // Phase 1: Move forward briefly (0-300ms)
  if(searchDuration < 300) {
    Serial.println("🔍 Search Phase 1: Moving forward");
    moveForward(baseSpeed/2);
  }
  // Phase 2: Zigzag based on last direction (300-1500ms)
  else if(searchDuration < 1500) {
    int zigzagPhase = ((searchDuration - 300) / 150) % 2;

    if(lastDirection == DIR_LEFT) {
      if(zigzagPhase == 0) {
        Serial.println("🔍 Search Phase 2: Turning left");
        turnLeft(baseSpeed);
      } else {
        moveForward(baseSpeed/2);
      }
    } else if(lastDirection == DIR_RIGHT) {
      if(zigzagPhase == 0) {
        Serial.println("🔍 Search Phase 2: Turning right");
        turnRight(baseSpeed);
      } else {
        moveForward(baseSpeed/2);
      }
    } else {
      // No clear last direction - alternate
      if(zigzagPhase == 0) {
        turnLeft(baseSpeed);
      } else {
        turnRight(baseSpeed);
      }
    }
  }
  // Phase 3: Backup and turn (1500-2000ms)
  else if(searchDuration < 2000) {
    Serial.println("🔍 Search Phase 3: Reversing");
    moveBackward(baseSpeed);
  }
  else if(searchDuration < 2500) {
    Serial.println("🔍 Search Phase 3: Wide turn");
    if(lastDirection == DIR_LEFT) {
      turnLeft(baseSpeed);
    } else {
      turnRight(baseSpeed);
    }
  }
  // Phase 4: Give up (2500ms+)
  else {
    Serial.println("❌ Line search failed - Stopping auto mode");
    stopMotors();
    autoMode = false;
    isSearching = false;
  }
}

/* =====================================================
   AUTO LINE FOLLOW WITH OBSTACLE AVOIDANCE & RECOVERY
   ===================================================== */
void lineFollowMode(){
  // Check ultrasonic for obstacle avoidance
  long ultraDist = getUltrasonicDistance();
  if(ultraDist < 20 && ultraDist > 0) {
    Serial.print("⚠️ Obstacle detected at ");
    Serial.print(ultraDist);
    Serial.println(" cm - Avoiding");

    stopMotors();
    delay(300);
    moveBackward(baseSpeed);
    delay(400);
    turnRight(baseSpeed);
    delay(500);
    stopMotors();
    delay(200);
    return;
  }

  // Read IR sensors
  int L = digitalRead(SENSOR_LEFT);
  int C = digitalRead(SENSOR_CENTER);
  int R = digitalRead(SENSOR_RIGHT);

  // All on black (intersection/stop) - Trigger scanning
  if(L==1 && C==1 && R==1) {
    scanAndWater();
    return;
  }

  // Normal line following with direction tracking
  if(L==0 && C==1 && R==0) {
    moveForward(baseSpeed);
    lastDirection = DIR_FORWARD;
    isSearching = false;
  }
  else if(L==1) {
    turnLeft(baseSpeed);
    lastDirection = DIR_LEFT;
    isSearching = false;
  }
  else if(R==1) {
    turnRight(baseSpeed);
    lastDirection = DIR_RIGHT;
    isSearching = false;
  }
  else if(L==0 && C==0 && R==0) {
    // Lost line - start recovery search
    if(!isSearching) {
      lostLineTime = millis();
      isSearching = true;
      Serial.println("⚠️ Line lost - Starting recovery search");
    }

    performLineSearch();
  }
}

/* =====================================================
   WEB PAGE
   ===================================================== */
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Rover</title>

<style>
body{
  text-align:center;
  font-family:Arial;
  background:#0e0e0e;
  color:#fff;
  margin:0;
  padding:10px;
}
h2{
  color:#00e676;
  font-size:22px;
  margin:12px 0;
}

/* ===== Circular movement buttons ===== */
.ctrl-btn{
  margin:6px;
  width:75px;
  height:75px;
  font-size:22px;
  border:none;
  border-radius:50%;
  background:#00bfa5;
  color:white;
  box-shadow:0 0 10px #00bfa5;
  transition:0.15s;
  cursor:pointer;
}
.ctrl-btn:active{
  background:#00796b;
  box-shadow:0 0 20px #00796b;
}

/* ===== Rectangular buttons ===== */
.rect-btn{
  width:220px;
  height:55px;
  font-size:18px;
  margin:10px;
  border:none;
  border-radius:12px;
  background:#ff9800;
  color:black;
  font-weight:bold;
  transition:0.2s;
  cursor:pointer;
}
.rect-btn.active{
  background:#4caf50;
  color:white;
}
.rect-btn:hover{
  opacity:0.9;
}

/* ===== Sliders ===== */
.slider{
  width:80%;
  margin:10px 0;
}

/* ===== Info card ===== */
.card{
  margin:12px auto;
  padding:12px;
  border:1px solid #00bfa5;
  border-radius:10px;
  max-width:400px;
  background:#1a1a1a;
}
.card span{
  color:#00e676;
  font-weight:bold;
}
</style>
</head>

<body>

<h2>🚗 ESP32 Smart Rover Dashboard</h2>

<div>
  <button class="ctrl-btn" onclick="manual('forward')">▲</button>
</div>

<div>
  <button class="ctrl-btn" onclick="manual('left')">◀</button>
  <button class="ctrl-btn" onclick="manual('stop')">■</button>
  <button class="ctrl-btn" onclick="manual('right')">▶</button>
</div>

<div>
  <button class="ctrl-btn" onclick="manual('back')">▼</button>
</div>

<!-- AUTO / MANUAL -->
<button id="modeBtn" class="rect-btn" onclick="toggleMode()">MANUAL MODE</button>

<p id="modeText">Current Mode: <strong>MANUAL</strong></p>

<!-- SPEED -->
<p>Speed: <span id="speedVal">200</span></p>
<input type="range" min="120" max="255" value="200" class="slider"
       oninput="updateSpeed(this.value)">

<!-- SERVO CONTROLS -->
<p>Servo X: <span id="servoXval">90</span>°</p>
<input type="range" min="0" max="180" value="90" class="slider"
       oninput="updateServoX(this.value)">

<p>Servo Y: <span id="servoYval">90</span>°</p>
<input type="range" min="0" max="180" value="90" class="slider"
       oninput="updateServoY(this.value)">

<!-- RELAY -->
<button class="rect-btn" onclick="cmd('relay')">💦 RELAY ON / OFF</button>

<!-- LIVE STATUS -->
<div class="card">
  <strong>📊 Live Sensor Data</strong><br><br>
  Moisture: <span id="moist">--</span><br>
  Relay: <span id="relay">--</span><br>
  Ultrasonic: <span id="ultradist">--</span> cm<br>
  Distance: <span id="dist">--</span> cm
</div>

<script>
let autoMode = false;

function cmd(c){
  fetch('/'+c);
}

function manual(cmdName){
  fetch('/'+cmdName);
}

function toggleMode(){
  fetch('/toggleMode')
    .then(r => r.text())
    .then(txt => {
      autoMode = (txt === "AUTO");
      document.getElementById("modeBtn").innerText =
        autoMode ? "🤖 AUTO MODE" : "🎮 MANUAL MODE";
      document.getElementById("modeText").innerHTML =
        "Current Mode: <strong>" + (autoMode ? "AUTO" : "MANUAL") + "</strong>";

      document.getElementById("modeBtn").classList.toggle("active", autoMode);
    });
}

function updateSpeed(v){
  document.getElementById("speedVal").innerText = v;
  fetch('/speed?val=' + v);
}

function updateServoX(v){
  document.getElementById("servoXval").innerText = v;
  fetch('/servoX?val=' + v);
}

function updateServoY(v){
  document.getElementById("servoYval").innerText = v;
  fetch('/servoY?val=' + v);
}

/* Live sensor update */
setInterval(() => {
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById("moist").innerText = d.moisture;
    document.getElementById("relay").innerText = d.relay;
    document.getElementById("ultradist").innerText = d.ultrasonic;
    document.getElementById("dist").innerText = d.distance;
  });
}, 1000);
</script>

</body>
</html>
)rawliteral";
}

/* =====================================================
   WEB HANDLERS
   ===================================================== */
void handleRoot(){ server.send(200,"text/html; charset=UTF-8",htmlPage()); }

void handleForward(){ if(!autoMode) moveForward(baseSpeed); server.send(200,"text/plain","OK"); }
void handleBack(){ if(!autoMode) moveBackward(baseSpeed); server.send(200,"text/plain","OK"); }
void handleLeft(){ if(!autoMode) turnLeft(baseSpeed); server.send(200,"text/plain","OK"); }
void handleRight(){ if(!autoMode) turnRight(baseSpeed); server.send(200,"text/plain","OK"); }
void handleStop(){ if(!autoMode) stopMotors(); server.send(200,"text/plain","OK"); }

void handleToggleMode(){
  autoMode=!autoMode;
  stopMotors();
  Serial.print("🔄 Mode switched to: ");
  Serial.println(autoMode ? "AUTO" : "MANUAL");
  server.send(200,"text/plain",autoMode?"AUTO":"MANUAL");
}

void handleSpeed(){
  if(server.hasArg("val"))
    baseSpeed=constrain(server.arg("val").toInt(),120,255);
  server.send(200,"text/plain","OK");
}

void handleServoX(){
  servoXAngle=server.arg("val").toInt();
  moveServo(SERVO_CH_X,servoXAngle);
  server.send(200,"text/plain","OK");
}
void handleServoY(){
  servoYAngle=server.arg("val").toInt();
  moveServo(SERVO_CH_Y,servoYAngle);
  server.send(200,"text/plain","OK");
}

void handleRelay(){
  digitalWrite(RELAY_PIN,!digitalRead(RELAY_PIN));
  server.send(200,"text/plain","OK");
}

void handleStatus(){
  String json="{";
  json+="\"moisture\":"+String(analogRead(MOISTURE_PIN))+",";
  json+="\"relay\":\"";
  json+=(digitalRead(RELAY_PIN)==HIGH?"ON":"OFF");
  json+="\",";
  json+="\"ultrasonic\":"+String(getUltrasonicDistance())+",";
  json+="\"distance\":"+String(getDistanceCM());
  json+="}";
  server.send(200,"application/json",json);
}

/* =====================================================
   SETUP
   ===================================================== */
void setup(){
  Serial.begin(115200);
  Serial.println("\n\n🚀 ESP32 Smart Rover Starting...\n");

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(SENSOR_LEFT,INPUT);
  pinMode(SENSOR_CENTER,INPUT);
  pinMode(SENSOR_RIGHT,INPUT);
  pinMode(RELAY_PIN,OUTPUT);
  pinMode(WIFI_LED,OUTPUT);
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  digitalWrite(RELAY_PIN,LOW);

  setupMotorPWM();

  ledc_timer_config_t st={LEDC_HIGH_SPEED_MODE,SERVO_RES,SERVO_TIMER,SERVO_FREQ,LEDC_AUTO_CLK};
  ledc_timer_config(&st);
  ledc_channel_config_t chX={SERVO_X,LEDC_HIGH_SPEED_MODE,SERVO_CH_X,LEDC_INTR_DISABLE,SERVO_TIMER,0,0};
  ledc_channel_config_t chY={SERVO_Y,LEDC_HIGH_SPEED_MODE,SERVO_CH_Y,LEDC_INTR_DISABLE,SERVO_TIMER,0,0};
  ledc_channel_config(&chX);
  ledc_channel_config(&chY);

  // Initialize servos to home position
  moveServo(SERVO_CH_X, 90);
  moveServo(SERVO_CH_Y, 90);
  Serial.println("✅ Servos initialized");

  Wire.begin(21,22);
  if(!lox.begin()) {
    Serial.println("❌ VL53L0X not found!");
  } else {
    Serial.println("✅ VL53L0X initialized");
  }

  // WIFI CONNECTION WITH BLINKING LED
  Serial.print("🌐 Connecting to WiFi");
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){
    digitalWrite(WIFI_LED,!digitalRead(WIFI_LED));  // BLINK while connecting
    delay(300);
    Serial.print(".");
  }
  digitalWrite(WIFI_LED,HIGH);  // SOLID when connected
  Serial.println(" Connected!");

  Serial.println("\n📡 Network Information:");
  Serial.print("   IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("   Access URL: http://");
  Serial.println(WiFi.localIP());

  // START WEBSERVER
  server.on("/",handleRoot);
  server.on("/forward",handleForward);
  server.on("/back",handleBack);
  server.on("/left",handleLeft);
  server.on("/right",handleRight);
  server.on("/stop",handleStop);
  server.on("/toggleMode",handleToggleMode);
  server.on("/speed",handleSpeed);
  server.on("/servoX",handleServoX);
  server.on("/servoY",handleServoY);
  server.on("/relay",handleRelay);
  server.on("/status",handleStatus);
  server.begin();
  Serial.println("✅ Web Server started on port 80");

  Serial.println("\n🎉 All Systems Ready!");
  Serial.println("═══════════════════════════════════════");
  Serial.println("Features Enabled:");
  Serial.println("  ✓ Line Following with Recovery");
  Serial.println("  ✓ Obstacle Avoidance");
  Serial.println("  ✓ Target Scanning");
  Serial.println("  ✓ Auto Watering System");
  Serial.println("  ✓ Non-blocking Mode Switch");
  Serial.println("═══════════════════════════════════════\n");
}

/* =====================================================
   LOOP
   ===================================================== */
void loop(){
  server.handleClient();  // Handle web requests

  if(autoMode) lineFollowMode();
}
