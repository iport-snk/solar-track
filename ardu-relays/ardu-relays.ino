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

// 🧭 Azimuth position tracking
uint8_t AZ_SENSOR_PIN = A7;
float azimuth_degrees = 0.0;
float target_azimuth = -1.0;  // Target azimuth (-1 means no target)
bool auto_azimuth_active = false;  // Whether auto-azimuth movement is active
float azimuth_tolerance = 2.0;  // Tolerance in degrees for reaching target

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
  unsigned long STATE_TIME;  // Use unsigned long to match millis() return type
  MotorState state;
  ERRORS err;
};

// 🧵 Motor array: Elevation and Azimuth
MotorSettings MOTORS[] = {
  {1, A0, 10, 9, 10, 5, 200, 0, 0, IDLE, ERRORS::NONE},     // Elevation
  {2, A1, 6, 5, 10, 5, 200, 0, 0, IDLE, ERRORS::NONE}      // Azimuth
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

// 🔁 Main loop
void loop() {
  now = millis();
  
  readAzimuthPosition();
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
  if (strcmp(input, "ping") == 0)           Serial.println("pong 1.20");
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
  else if (strncmp(input, "AZ:", 3) == 0)   azMove(input);
  else if (strncmp(input, "CMD:", 4) == 0)  execCmd(input);
  //else if (strcmp(input, "M1_0") == 0)      elCurr = 12;
  else {
    Serial.print("Unknown command: ");
    Serial.println(input);
  }
}

void execCmd(const char* input) {
  // Skip "CMD:"
  const char* ptr = input + 4;

  // Find first delimiter
  const char* delim = strchr(ptr, ':');
  if (!delim) return;

  // Extract command code
  size_t cmdLen = delim - ptr;
  char cmdCode[16] = {};
  strncpy(cmdCode, ptr, cmdLen);

  // Extract ID
  const char* idStr = delim + 1;

  char buff[64];
  sprintf(buff, "ACK:%s:", idStr);
  RelayState(buff);
  Serial.write(STF_BYTE);
  Serial.println(buff);
}

void RelayState(char* to) {
  // Show motor states: 1=CW active, 0=inactive for each motor (EL_CW, EL_CCW, AZ_CW, AZ_CCW)
  sprintf(to + strlen(to), "%d%d%d%d", 
    (MOTORS[0].state == MOVING_CW) ? 1 : 0,
    (MOTORS[0].state == MOVING_CCW) ? 1 : 0,
    (MOTORS[1].state == MOVING_CW) ? 1 : 0,
    (MOTORS[1].state == MOVING_CCW) ? 1 : 0
  );
}

// 🔄 Reboot system
void Reboot() {
  Serial.println("Rebooting...");
  for (uint8_t i = 0; i < 2; i++) transitionState(&MOTORS[i], IDLE, ERRORS::NONE);
  asm volatile ("jmp 0");
}

// 🌀 Start motor with PWM at 98% duty cycle
void StartMotor(MotorSettings* motor, bool CW) {
  analogWrite(motor->CW_PIN,  CW ? 253 : 0);
  analogWrite(motor->CCW_PIN, CW ? 0 : 253);
}

// 🛑 Stop motor (set PWM to 0)
void StopMotor(MotorSettings* motor) {
  analogWrite(motor->CW_PIN, 0);
  analogWrite(motor->CCW_PIN, 0);
}

// 🔁 Transition motor state
void transitionState(MotorSettings* motor, MotorState next, ERRORS err) {
  // GUARDS
  motor->err = err;
  if (motor->state == ERROR) return;
  if (next == MOVING_CW && motor->state == RICHED_CW) return;
  if (next == MOVING_CCW && motor->state == RICHED_CCW) return;

  motor->state = next;
  // Overflow-safe duration calculation using unsigned arithmetic
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
void readAzimuthPosition() {
  int raw = analogRead(AZ_SENSOR_PIN);
  azimuth_degrees = (raw / 1023.0) * 360.0;
  
  // Check if we need to stop at target azimuth
  if (auto_azimuth_active && target_azimuth >= 0) {
    bool should_stop = false;
    
    // Check if we've reached or passed the target based on motor direction
    if (MOTORS[1].state == MOVING_CW) {
      // Moving clockwise: stop if we've reached or passed target
      should_stop = (azimuth_degrees >= target_azimuth - azimuth_tolerance);
    } else if (MOTORS[1].state == MOVING_CCW) {
      // Moving counter-clockwise: stop if we've reached or passed target  
      should_stop = (azimuth_degrees <= target_azimuth + azimuth_tolerance);
    }
    
    if (should_stop) {
      // Stop the azimuth motor
      transitionState(&MOTORS[1], IDLE, ERRORS::NONE);
      auto_azimuth_active = false;
      target_azimuth = -1.0;
      
      // Report that target was reached
      char pos_str[8];
      dtostrf(azimuth_degrees, 6, 2, pos_str);
      Serial.print("AZ:TARGET_REACHED:");
      Serial.println(pos_str);
    }
  }
}

// 📍 Report current azimuth position
void reportAzimuthPosition() {
  char pos_str[8];
  dtostrf(azimuth_degrees, 6, 2, pos_str);  // Format: "123.45"
  Serial.print("AZPOS:");
  Serial.println(pos_str);
}

// 🎯 Move azimuth to target angle
void azMove(const char* input) {
  // Parse target angle from "AZ:120" format
  const char* angle_str = input + 3;  // Skip "AZ:"
  target_azimuth = atof(angle_str);
  
  // Check if target is within valid range (40-340 degrees)
  if (target_azimuth < 40 || target_azimuth > 340) {
    Serial.println("AZ:ERROR_OUT_OF_RANGE");
    return;
  }
  
  // Check if already at target
  float diff = fabs(target_azimuth - azimuth_degrees);
  if (diff <= azimuth_tolerance) {
    Serial.println("AZ:TARGET_REACHED");
    return;
  }
  
  auto_azimuth_active = true;
  
  // Simple direction logic: if target > current, go CW; if target < current, go CCW
  if (target_azimuth > azimuth_degrees) {
    // Move CW to reach target
    transitionState(&MOTORS[1], MOVING_CW, ERRORS::NONE);
    Serial.print("AZ:MOVING_CW_TO:");
  } else {
    // Move CCW to reach target
    transitionState(&MOTORS[1], MOVING_CCW, ERRORS::NONE);
    Serial.print("AZ:MOVING_CCW_TO:");
  }
  
  char target_str[8];
  dtostrf(target_azimuth, 6, 2, target_str);
  Serial.println(target_str);
}

// 🔍 Read current in amps
void readCurrentAmps(MotorSettings* motor) {
  int raw = analogRead(motor->CURR_PIN);
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

// 🛡️ Guard motor against faults
void guardMotor(MotorSettings* motor) {
  if (!(motor->state == MOVING_CW || motor->state == MOVING_CCW)) return;

  readCurrentAmps(motor);
  
  // Overflow-safe elapsed time calculation
  unsigned long elapsed = now - motor->STATE_TIME;  // Direct unsigned subtraction

  // With PWM, we track direction via motor state instead of digitalRead
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
