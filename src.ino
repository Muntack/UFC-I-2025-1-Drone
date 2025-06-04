//Código
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Pines PWM para motores
const int ena = 3;
const int enb = 5;
const int enc = 6;
const int end = 9;

// Pines de dirección
const int in1 = 22, in2 = 23;
const int in3 = 24, in4 = 25;
const int in5 = 26, in6 = 27;
const int in7 = 28, in8 = 29;

// Potenciómetro y botón
const int pinPot = A0;
const int pinBoton = 4;

bool mantenerAltura = false;
int basePWM = 140;
int potenciaFijada = 140;

int16_t ax, ay, az, gx, gy, gz;
float roll = 0, pitch = 0;
unsigned long lastTime = 0;

float Kp = 1.5;
float alpha = 0.98;

// Valores de referencia para hover
const int16_t gravedadCruda = 16384;
const int16_t tolerancia = 1000;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no conectado.");
    while (1);
  }

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT); pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT); pinMode(in8, OUTPUT);

  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH); digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH); digitalWrite(in8, LOW);

  pinMode(ena, OUTPUT); pinMode(enb, OUTPUT);
  pinMode(enc, OUTPUT); pinMode(end, OUTPUT);

  pinMode(pinBoton, INPUT_PULLUP);
  lastTime = millis();
}

void loop() {
  // Leer sensor
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  actualizarAngulos(ax, ay, az, gx, gy);

  // Leer potenciómetro
  int valorPot = analogRead(pinPot);
  int pwmPot = map(valorPot, 0, 1023, 100, 200);

  // Detectar botón presionado
  static bool botonPrevio = HIGH;
  bool botonActual = digitalRead(pinBoton);
  if (botonPrevio == HIGH && botonActual == LOW) {
    if (abs(az - gravedadCruda) < tolerancia) {
      // Dron está quieto → mantener altura
      mantenerAltura = true;
      potenciaFijada = basePWM;
    } else {
      // Está subiendo o bajando → ajustar hasta que se estabilice
      mantenerAltura = true;
      potenciaFijada = pwmPot; // punto de partida
    }
    delay(300); // Anti rebote
  }
  botonPrevio = botonActual;

  // Ver si se debe recuperar control manual
  if (mantenerAltura && abs(pwmPot - potenciaFijada) > 5) {
    mantenerAltura = false;
  }

  // Ajustar basePWM según modo
  if (mantenerAltura) {
    if (az < (gravedadCruda - tolerancia)) {
      potenciaFijada += 1; // Subiendo rápido → bajamos empuje
    } else if (az > (gravedadCruda + tolerancia)) {
      potenciaFijada -= 1; // Bajando rápido → subimos empuje
    }
    potenciaFijada = constrain(potenciaFijada, 100, 200);
    basePWM = potenciaFijada;
  } else {
    basePWM = pwmPot;
  }

  // Estabilización con Roll/Pitch
  float errorRoll = 0 - roll;
  float errorPitch = 0 - pitch;
  int correccionRoll = errorRoll * Kp;
  int correccionPitch = errorPitch * Kp;

  int pwm1 = basePWM - correccionPitch - correccionRoll;
  int pwm2 = basePWM - correccionPitch + correccionRoll;
  int pwm3 = basePWM + correccionPitch + correccionRoll;
  int pwm4 = basePWM + correccionPitch - correccionRoll;

  analogWrite(ena, constrain(pwm1, 0, 255));
  analogWrite(enb, constrain(pwm2, 0, 255));
  analogWrite(enc, constrain(pwm3, 0, 255));
  analogWrite(end, constrain(pwm4, 0, 255));

  Serial.print("Roll: "); Serial.print(roll, 1);
  Serial.print(" | Pitch: "); Serial.print(pitch, 1);
  Serial.print(" | PWM Base: "); Serial.print(basePWM);
  Serial.print(" | az: "); Serial.println(az);

  delay(20);
}

void actualizarAngulos(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float accRoll = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float gyroRoll = gx * dt / 131.0;
  float gyroPitch = gy * dt / 131.0;

  roll = alpha * (roll + gyroRoll) + (1 - alpha) * accRoll;
  pitch = alpha * (pitch + gyroPitch) + (1 - alpha) * accPitch;
}
