#include <PS2X_lib.h>
#include <Wire.h>

#define PS2_DAT 22
#define PS2_CMD 23
#define PS2_SEL 26
#define PS2_CLK 33
#define pressures false
#define rumble    false

#define I2C_SDA 18
#define I2C_SCL 19

#define PCA9685_ADDR  0x40
#define MODE1         0x00
#define PRESCALE_REG  0xFE
#define L_IN1  12
#define L_IN2  13
#define R_IN1  14
#define R_IN2  15

#define DEAD_ZONE  20
#define MIN_PWM   200

PS2X ps2x;
int  error   = -1;
byte type    = 0;
byte vibrate = 0;
int  tryNum  = 1;
bool expansionReady = false;

// ── I2C recovery ──────────────────────────────────────────────
void i2c_recover() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    if (digitalRead(I2C_SDA)) break;
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
  }
  pinMode(I2C_SDA, OUTPUT);
  digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);
  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
}

// ── PCA9685 helpers ───────────────────────────────────────────
void pca_write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission(true) != 0) i2c_recover();
}

uint8_t pca_read(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2c_recover(); return 0; }
  Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

void pca_setPWM(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on  & 0xFF); Wire.write(on  >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  if (Wire.endTransmission(true) != 0) i2c_recover();
}

void pca_duty(uint8_t ch, uint16_t value) {
  value = constrain(value, 0, 4095);
  if      (value == 0)    pca_setPWM(ch, 0,    4096);
  else if (value == 4095) pca_setPWM(ch, 4096, 0);
  else                    pca_setPWM(ch, 0,    value);
}

void pca_setFreq(float freq) {
  uint8_t prescale = (uint8_t)(25000000.0f / 4096.0f / freq + 0.5f);
  uint8_t old_mode = pca_read(MODE1);
  pca_write(MODE1, (old_mode & 0x7F) | 0x10);
  pca_write(PRESCALE_REG, prescale);
  pca_write(MODE1, old_mode);
  delayMicroseconds(500);
  pca_write(MODE1, old_mode | 0xA1);
  delay(1);
}

bool pca_init() {
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("PCA9685 NOT found!");
    return false;
  }
  pca_write(MODE1, 0x00);
  delay(10);
  pca_write(0xFA, 0x00);
  pca_write(0xFB, 0x00);
  pca_write(0xFC, 0x00);
  pca_write(0xFD, 0x10);
  pca_setFreq(50);
  delay(10);
  for (uint8_t i = 0; i < 16; i++) pca_duty(i, 0);
  Serial.println("Expansion Board ready.");
  return true;
}

// ── Motor drivers ─────────────────────────────────────────────
void motorL(uint16_t speed, bool fwd) {
  if (speed < MIN_PWM) { pca_duty(L_IN1, 0); pca_duty(L_IN2, 0); return; }
  pca_duty(L_IN1, fwd ? 0 : speed);
  pca_duty(L_IN2, fwd ? speed : 0);
}

void motorR(uint16_t speed, bool fwd) {
  if (speed < MIN_PWM) { pca_duty(R_IN1, 0); pca_duty(R_IN2, 0); return; }
  pca_duty(R_IN1, fwd ? 0 : speed);
  pca_duty(R_IN2, fwd ? speed : 0);
}

void stopMotors() {
  pca_duty(L_IN1, 0); pca_duty(L_IN2, 0);
  pca_duty(R_IN1, 0); pca_duty(R_IN2, 0);
}

// ── Controller connection check ───────────────────────────────
// PS2 returns 255,255 on ALL axes when signal is lost
// Also catches 0,0 which some receivers output on disconnect
bool isControllerConnected(int lx, int ly) {
  if (lx == 255 && ly == 255) return false;
  if (lx == 0   && ly == 0)   return false;
  return true;
}

// ── R-Theta mixing ────────────────────────────────────────────
void rThetaMix(int lx, int ly, int &leftOut, int &rightOut) {
  float x = -(lx - 128);
  float y = -(ly - 128);
  if (abs(x) < DEAD_ZONE) x = 0;
  if (abs(y) < DEAD_ZONE) y = 0;
  if (x == 0 && y == 0) { leftOut = 0; rightOut = 0; return; }
  float r     = constrain(sqrt(x*x + y*y), 0, 128);
  float theta = atan2(y, x);
  leftOut  = (int)(r * sin(theta + PI/4.0f) * (4095.0f/128.0f));
  rightOut = (int)(r * sin(theta - PI/4.0f) * (4095.0f/128.0f));
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Boot ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);
  expansionReady = pca_init();

  while (error != 0) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("PS2 try #"); Serial.print(tryNum++);
    Serial.print(" err="); Serial.println(error);
  }

  type = ps2x.readType();
  switch (type) {
    case 0: Serial.println("Unknown controller");       break;
    case 1: Serial.println("DualShock found");          break;
    case 2: Serial.println("GuitarHero found");         break;
    case 3: Serial.println("Wireless DualShock found"); break;
  }
  Serial.println("=== Ready ===");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  if (!expansionReady) {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    expansionReady = pca_init();
    delay(1000);
    return;
  }

  if (type == 1) {
    ps2x.read_gamepad(false, vibrate);

    int lx = ps2x.Analog(PSS_LX);
    int ly = ps2x.Analog(PSS_LY);

    // ── DISCONNECT GUARD ──────────────────────────────────────
    // LX=255 LY=255 means receiver lost signal — stop everything
    if (!isControllerConnected(lx, ly)) {
      stopMotors();
      Serial.println("CONTROLLER DISCONNECTED — motors stopped");
      delay(100);
      return;
    }

    // ── Always stop before deciding ───────────────────────────
    stopMotors();

    if (ps2x.Button(PSB_SELECT)) {
      // stay stopped

    } else if (ps2x.Button(PSB_PAD_UP)) {
      motorL(4095, true);  motorR(4095, true);
      Serial.println("FWD");

    } else if (ps2x.Button(PSB_PAD_DOWN)) {
      motorL(4095, false); motorR(4095, false);
      Serial.println("BWD");

    } else if (ps2x.Button(PSB_PAD_LEFT)) {
      motorL(4095, true);  motorR(4095, false);

    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
      motorL(4095, false); motorR(4095, true);

    } else {
      int leftVal = 0, rightVal = 0;
      rThetaMix(lx, ly, leftVal, rightVal);

      Serial.print("LX="); Serial.print(lx);
      Serial.print(" LY="); Serial.print(ly);
      Serial.print(" L="); Serial.print(leftVal);
      Serial.print(" R="); Serial.println(rightVal);

      motorL((uint16_t)abs(leftVal), leftVal  >= 0);
      motorR((uint16_t)abs(rightVal), rightVal >= 0);
    }

    vibrate = ps2x.Analog(PSAB_CROSS);
    if (ps2x.ButtonPressed(PSB_START)) Serial.println("Start");
  }

  delay(50);
}