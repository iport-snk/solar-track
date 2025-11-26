// 50 - 310 : azimuth border limits
int16_t azimuth_offset = 152; // Calibration offset if needed

// TESTS
float elCurr = 6;
float azCurr = 6;
// END TESTS
#define STF_BYTE 0xAA
bool dump_amp = false;
unsigned long now;  // Use unsigned long to match millis() return type
// 🧭 Motor state definitions
enum MotorState {
  IDLE,
  MOVING_CW,
  MOVING_CCW,
  RICHED_CW,
  RICHED_CCW,
  ERROR
};
enum ERRORS {
  NONE,
  OVERCURR,
  TWODIRS
};

uint8_t VOLT_SENSOR_PIN = A6;

uint8_t AZ_SENSOR_PIN = A7;
uint16_t tolerance = 2;  // Tolerance in degrees for reaching target


// �🧰 Motor configuration struct
struct MotorSettings {
  uint8_t ID;
  uint8_t CURR_PIN;
  uint8_t CW_PIN;
  uint8_t CCW_PIN;
  float OVERCURR;
  float CURR_MIN;
  uint16_t CURR_BURST_MS;
  float CURR;
  uint16_t angle_current;        // Current angle
  uint16_t angle_target;         // Target angle
  uint16_t angle_min;            // Minimum allowed angle
  uint16_t angle_max;            // Maximum allowed angle
  bool auto_move;             // Whether auto-move is active
  unsigned long STATE_TIME;   // Use unsigned long to match millis() return type
  MotorState state;
  ERRORS err;
};

// 🧵 Motor array: Elevation and Azimuth
MotorSettings MOTORS[] = {
  {1, A0, 9, 10, 10, 5, 200, 0, -1, -1, 0,  90,  false, 0, IDLE, ERRORS::NONE},     // Elevation
  {2, A1, 6,  5, 10, 5, 200, 0, -1, -1, 45, 315, false, 0, IDLE, ERRORS::NONE}      // Azimuth
};

// 📟 Serial command buffer
const int CMD_BUF_SIZE = 64;
char cmd[CMD_BUF_SIZE];
int cmdIndex = 0;

// 🏷️ Convert state to string
const char* prnState(MotorSettings* motor, unsigned long duration) {
  char prn[64];
  
  const char* result = nullptr;
  switch (motor->state) {
    case IDLE:        result = "IDLE"; break;
    case MOVING_CW:   result = "MOVING_CW"; break;
    case MOVING_CCW:  result = "MOVING_CCW"; break;
    case RICHED_CW:   result = "RICHED_CW"; break;
    case RICHED_CCW:  result = "RICHED_CCW"; break;
    case ERROR:       result = "ERR"; break;
    default:          result = "UNKNOWN"; break;
  }

  sprintf(prn, "%d:STATE:%s:%d:%d", motor->ID, result, motor->err, duration);
  Serial.println(prn);
}

// 🚀 Setup routine
void setup() {
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  now = millis();  // Initialize timing
  

  
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(MOTORS[i].CW_PIN, OUTPUT);
    pinMode(MOTORS[i].CCW_PIN, OUTPUT);

    // Create the falling edge to reset the relay
    digitalWrite(MOTORS[i].CCW_PIN, HIGH);   
    digitalWrite(MOTORS[i].CW_PIN, HIGH);
    delay(10);
    analogWrite(MOTORS[i].CW_PIN, 0);
    analogWrite(MOTORS[i].CCW_PIN, 0);
    delay(1);

    // Initialize STATE_TIME to current time to avoid huge boot duration
    MOTORS[i].STATE_TIME = now;
    transitionState(&MOTORS[i], IDLE, ERRORS::NONE);
  }
}

uint16_t readAzimuth() {
  float sensor_degrees = (analogRead(AZ_SENSOR_PIN) / 1023.0) * 360.0;
  return ((int16_t)sensor_degrees - azimuth_offset + 360) % 360;
}

void readVoltage() {
  float sensor_voltage = (analogRead(VOLT_SENSOR_PIN) / 1023.0) * 5.25;
  Serial.print("Voltage: ");
  Serial.println(sensor_voltage);
}



// 🔁 Main loop
void loop() {
  now = millis();
  MOTORS[1].angle_current =  readAzimuth();

  loopMotorPosition(&MOTORS[0]);
  loopMotorPosition(&MOTORS[1]);
  readSerialInput();
  runStateMachine();
}

// 📥 Read serial input
void readSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmd[cmdIndex] = '\0';
        routeCommand(cmd);
        cmdIndex = 0;
      }
    } else if (cmdIndex < CMD_BUF_SIZE - 1) {
      cmd[cmdIndex++] = c;
    } else {
      cmdIndex = 0;
    }
  }
}


// 🧭 Route commands
void routeCommand(const char* input) {
  if (strcmp(input, "ping") == 0)           Serial.println("pong 1.23");
  else if (strcmp(input, "reboot") == 0)    Reboot();
  else if (strcmp(input, "ELCW") == 0)      transitionState(&MOTORS[0], MOVING_CW, ERRORS::NONE);
  else if (strcmp(input, "ELCCW") == 0)     transitionState(&MOTORS[0], MOVING_CCW, ERRORS::NONE);
  else if (strcmp(input, "ELSTOP") == 0)    transitionState(&MOTORS[0], IDLE, ERRORS::NONE);
  else if (strcmp(input, "AZCW") == 0)      transitionState(&MOTORS[1], MOVING_CW, ERRORS::NONE);
  else if (strcmp(input, "AZCCW") == 0)     transitionState(&MOTORS[1], MOVING_CCW, ERRORS::NONE);
  else if (strcmp(input, "AZSTOP") == 0)    transitionState(&MOTORS[1], IDLE, ERRORS::NONE);
  else if (strcmp(input, "STOP") == 0)      for (uint8_t i = 0; i < 2; i++) transitionState(&MOTORS[i], IDLE, ERRORS::NONE);
  else if (strcmp(input, "PRNAMP1") == 0)   dump_amp = true;
  else if (strcmp(input, "PRNAMP0") == 0)   dump_amp = false;
  else if (strcmp(input, "AZPOS") == 0)     reportAzimuthPosition();
  else if (strcmp(input, "MOTORS") == 0)    reportMotorStates();
  else if (strncmp(input, "CMD:", 4) == 0)  execCmd(input);
  else if (strncmp(input, "SNR:", 4) == 0)  setSensor(input);
  //else if (strcmp(input, "M1_0") == 0)      elCurr = 12;
  else {
    Serial.print("Unknown command: ");
    Serial.println(input);
  }
}

void setSensor(const char* input) {
  const char* ptr = input + 4;            // Skip "SNR:"
  const char* delim = strchr(ptr, ':');
  if (!delim)  return;

  char sensorName[16] = {};
  strncpy(sensorName, ptr, delim - ptr);

  if (strcmp(sensorName, "EL") == 0) MOTORS[0].angle_current = atof(delim + 1);
}

void execCmd(const char* input) {
  // Skip "CMD:"
  const char* ptr = input + 4;

  const char* delim = strchr(ptr, ':');
  if (!delim) delim = ptr + strlen(ptr); // If no colon, point to end of string

  // Extract command code
  char cmdCode[16] = {};
  strncpy(cmdCode, ptr, delim - ptr);

  char buff[64];
  
  sprintf(buff, "ACK:%s:", cmdCode);
  if (strcmp(cmdCode, "MOTORS") == 0) {
    RelayState(buff);
  } else if (strcmp(cmdCode, "POZ") == 0) {
    sprintf(buff + strlen(buff), "%u~%u", (uint16_t)MOTORS[0].angle_current, (uint16_t)MOTORS[1].angle_current);
  } else if (strcmp(cmdCode, "AZ") == 0 || strcmp(cmdCode, "EL") == 0) {
    const char* targetAngleStr = delim + 1;
    if (strlen(targetAngleStr) > 0) motorMove(cmdCode, atof(targetAngleStr));
    return;
  }
  
  Serial.write(STF_BYTE);
  Serial.println(buff);
}

void reportMotorStates() {
    char buff[16];
    sprintf(buff, "MOTORS:");
    RelayState(buff);
    Serial.println(buff);
}

void RelayState(char* to) {
  sprintf(to + strlen(to), "%d%d%d%d", 
    (MOTORS[0].state == MOVING_CW) ? 1 : 0,
    (MOTORS[0].state == MOVING_CCW) ? 1 : 0,
    (MOTORS[1].state == MOVING_CW) ? 1 : 0,
    (MOTORS[1].state == MOVING_CCW) ? 1 : 0
  );
}

void Reboot() {
  Serial.println("Rebooting...");
  for (uint8_t i = 0; i < 2; i++) transitionState(&MOTORS[i], IDLE, ERRORS::NONE);
  asm volatile ("jmp 0");
}

void StartMotor(MotorSettings* motor, bool CW) {
  analogWrite(motor->CW_PIN,  CW ? 253 : 0);
  analogWrite(motor->CCW_PIN, CW ? 0 : 253);    // 98% duty cycle
}

void StopMotor(MotorSettings* motor) {
  analogWrite(motor->CW_PIN, 0);
  analogWrite(motor->CCW_PIN, 0);
}


void transitionState(MotorSettings* motor, MotorState next, ERRORS err) {
  motor->err = err;
  if (motor->state == ERROR) return;
  if (next == MOVING_CW && motor->state == RICHED_CW) return;
  if (next == MOVING_CCW && motor->state == RICHED_CCW) return;

  motor->state = next;
  unsigned long duration = now - motor->STATE_TIME;  // Direct unsigned subtraction
  motor->STATE_TIME = now;

  if (next == MOVING_CW || next == MOVING_CCW) {
    StartMotor(motor, next == MOVING_CW);
  } else {
    StopMotor(motor);
  }
  prnState(motor, duration);
}

// 🧭 Read azimuth position sensor
void loopMotorPosition(MotorSettings* motor) {
  // Check if we need to stop at target angle


  if (motor->auto_move && motor->angle_target >= 0) {
    bool should_stop = (motor->state == MOVING_CW) ? 
      (motor->angle_current >= motor->angle_target) : 
      (motor->angle_current <= motor->angle_target);
  
    if (should_stop) {
      // Stop the motor
      transitionState(motor, IDLE, ERRORS::NONE);
      motor->auto_move = false;
      motor->angle_target = -1;
      
      char msg[32];
      sprintf(msg, "ACK:%s:%u", motor->ID == 1 ? "EL" : "AZ", (uint16_t)(motor->angle_current));
      Serial.write(STF_BYTE);
      Serial.println(msg);
    }
  }
}

// 📍 Report current azimuth position
void reportAzimuthPosition() {
  char pos_str[8];
  dtostrf(MOTORS[1].angle_current, 6, 2, pos_str);  // Format: "123.45"
  Serial.print("AZPOS:");
  Serial.println(pos_str);
}

// 🎯 Move azimuth to target angle
void motorMove(char* cmd, uint16_t angle) {
  MotorSettings* motor = (strcmp(cmd, "AZ") == 0) ? &MOTORS[1] : &MOTORS[0];
  if (angle < motor->angle_min || angle > motor->angle_max) {
    char msg[32];
    sprintf(msg, "ACK:%s:OUT_OF_RANGE:%d:%d:%d", cmd, (uint16_t)angle, (uint16_t)motor->angle_min, (uint16_t)motor->angle_max);
    Serial.write(STF_BYTE);
    Serial.println(msg);
    return;
  }

  motor->angle_target = angle;
  motor->auto_move = true;
  transitionState(
    motor,
    motor->angle_target > motor->angle_current ? MOVING_CW : MOVING_CCW,
    ERRORS::NONE
  );
}


// 🔍 Read current in amps
void readCurrentAmps(MotorSettings* motor) {
  int raw = analogRead(motor->CURR_PIN);
  delay(10);
  raw = analogRead(motor->CURR_PIN);
  float voltage = raw * (5.0 / 1023.0);

  
  // motor->CURR = (voltage - 2.5) / 0.100;
  // TESTS
  motor->CURR = motor->CURR_PIN == A0 ? elCurr : azCurr;

  if (dump_amp) {
    char prn[64];
    char famp[8];
    dtostrf(motor->CURR, 5, 2, famp);
    sprintf(prn, "%d:AMP:%s", motor->ID, famp);
    Serial.println(prn);
  }
}

void guardMotor(MotorSettings* motor) {
  if (!(motor->state == MOVING_CW || motor->state == MOVING_CCW)) return;

  readCurrentAmps(motor);
  
  unsigned long elapsed = now - motor->STATE_TIME;  
  bool isMovingCW = (motor->state == MOVING_CW);
  bool isMovingCCW = (motor->state == MOVING_CCW);

  if (elapsed < motor->CURR_BURST_MS) return;

  if (motor->CURR > motor->OVERCURR) {
    transitionState(motor, ERROR, ERRORS::OVERCURR);
    return;
  }

  if (motor->CURR < motor->CURR_MIN) {
    transitionState(motor, isMovingCW ? RICHED_CW : RICHED_CCW, ERRORS::NONE);
    return;
  }
}

// 🧠 Run FSM logic
void runStateMachine() {
  bool allAligned = true;
  bool anyError = false;

  for (uint8_t i = 0; i < 2; i++)  guardMotor(&MOTORS[i]);
    //if (MOTORS[i].state == ERROR) anyError = true;
    //if (MOTORS[i].state != RICHED_CW && MOTORS[i].state != RICHED_CCW) allAligned = false;
}

// 🚨 Handle error state
void handleErrorState() {
  for (uint8_t i = 0; i < 2; i++) {
    // transitionState(&MOTORS[i], ERROR);
  }
  Serial.println("ERROR");
}
