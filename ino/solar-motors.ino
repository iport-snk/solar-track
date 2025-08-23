// TESTS
//float elCurr = 6;
//float azCurr = 6;
// END TESTS
#define STF_BYTE 0xAA
bool dump_amp = false;
uint32_t now = millis();
// 🧭 Motor state definitions
enum MotorState {
  IDLE,
  MOVING_CW,
  MOVING_CCW,
  RICHED_CW,
  RICHED_CCW,
  ERROR
};

// 🧰 Motor configuration struct
struct MotorSettings {
  uint8_t ID;
  uint8_t CURR_PIN;
  uint8_t CW_PIN;
  uint8_t CCW_PIN;
  float OVERCURR;
  float CURR_MIN;
  uint16_t CURR_BURST_MS;
  float CURR;
  unsigned long STATE_TIME;
  MotorState state;
};

// 🧵 Motor array: Elevation and Azimuth
MotorSettings MOTORS[] = {
  {1, A0, 4, 5, 10, 5, 200, 0, 0, IDLE},     // Elevation
  {2, A1, 6, 7, 10, 5, 200, 0, 0, IDLE}      // Azimuth
};

// 📟 Serial command buffer
const int CMD_BUF_SIZE = 64;
char cmd[CMD_BUF_SIZE];
int cmdIndex = 0;

// 🏷️ Convert state to string
const char* stateToString(MotorState s) {
  switch (s) {
    case IDLE: return "IDLE";
    case MOVING_CW: return "MOVING_CW";
    case MOVING_CCW: return "MOVING_CCW";
    case RICHED_CW: return "RICHED_CW";
    case RICHED_CCW: return "RICHED_CCW";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// 🚀 Setup routine
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(MOTORS[i].CW_PIN, OUTPUT);
    pinMode(MOTORS[i].CCW_PIN, OUTPUT);
    transitionState(&MOTORS[i], IDLE);
  }
}

// 🔁 Main loop
void loop() {
  now = millis();
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
  if (strcmp(input, "ping") == 0)           Serial.println("pong 1.17");
  else if (strcmp(input, "reboot") == 0)    Reboot();
  else if (strcmp(input, "ELCW") == 0)      transitionState(&MOTORS[0], MOVING_CW);
  else if (strcmp(input, "ELCCW") == 0)     transitionState(&MOTORS[0], MOVING_CCW);
  else if (strcmp(input, "ELSTOP") == 0)    transitionState(&MOTORS[0], IDLE);
  else if (strcmp(input, "AZCW") == 0)      transitionState(&MOTORS[1], MOVING_CW);
  else if (strcmp(input, "AZCCW") == 0)     transitionState(&MOTORS[1], MOVING_CCW);
  else if (strcmp(input, "AZSTOP") == 0)    transitionState(&MOTORS[1], IDLE);
  else if (strcmp(input, "STOP") == 0)      for (uint8_t i = 0; i < 2; i++) transitionState(&MOTORS[i], IDLE);
  else if (strcmp(input, "PRNAMP1") == 0)   dump_amp = true;
  else if (strcmp(input, "PRNAMP0") == 0)   dump_amp = false;
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
  uint32_t id = strtoul(idStr, nullptr, 16); // Accepts decimal or hex

  // Dispatch
  //Serial.print("CMD: "); Serial.println(cmdCode);
  //Serial.print("ID: "); Serial.println(id, HEX);

  Serial.write(STF_BYTE);         // Start-of-frame marker
  Serial.print("ACK:");
  Serial.print(idStr);          // Or HEX if preferred
  Serial.print(":payload\n");
}

// 🔄 Reboot system
void Reboot() {
  Serial.println("Rebooting...");
  for (uint8_t i = 0; i < 2; i++) transitionState(&MOTORS[i], IDLE);
  asm volatile ("jmp 0");
}

// 🌀 Start motor
void StartMotor(MotorSettings* motor, bool CW) {
  digitalWrite(motor->CW_PIN, CW ? HIGH : LOW);
  digitalWrite(motor->CCW_PIN, CW ? LOW : HIGH);
}

// 🛑 Stop motor
void StopMotor(MotorSettings* motor) {
  digitalWrite(motor->CW_PIN, LOW);
  digitalWrite(motor->CCW_PIN, LOW);
}

// 🔁 Transition motor state
void transitionState(MotorSettings* motor, MotorState next) {
  // GUARDS
  if (motor->state == ERROR) return;
  if (next == MOVING_CW && motor->state == RICHED_CW) return;
  if (next == MOVING_CCW && motor->state == RICHED_CCW) return;

  motor->state = next;
  uint32_t duration = now - motor->STATE_TIME;
  motor->STATE_TIME = now;

  if (next == MOVING_CW || next == MOVING_CCW) {
    StartMotor(motor, next == MOVING_CW);
  } else {
    StopMotor(motor);
  }

  char prn[64];
  sprintf(prn, "%d:STATE:%s:%d", motor->ID, stateToString(next), duration);
  Serial.println(prn);
}

// 🔍 Read current in amps
void readCurrentAmps(MotorSettings* motor) {
  int raw = analogRead(motor->CURR_PIN);
  float voltage = raw * (5.0 / 1023.0);

  
  motor->CURR = (voltage - 2.5) / 0.100;
  // TESTS
  // motor->CURR = motor->CURR_PIN == A0 ? elCurr : azCurr;

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
  
  uint32_t elapsed = now - motor->STATE_TIME;

  bool isMovingCW = digitalRead(motor->CW_PIN);
  bool isMovingCCW = digitalRead(motor->CCW_PIN);

  if (isMovingCW && isMovingCCW) {
    transitionState(motor, ERROR);
    return;
  }

  if (elapsed < motor->CURR_BURST_MS) return;

  if (motor->CURR > motor->OVERCURR) {
    transitionState(motor, ERROR);
    return;
  }

  if (motor->CURR < motor->CURR_MIN) {
    transitionState(motor, isMovingCW ? RICHED_CW : RICHED_CCW);
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
    transitionState(&MOTORS[i], ERROR);
  }
  Serial.println("ERROR");
}
