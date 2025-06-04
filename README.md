# UFC-I-2025-1-Drone
Código inicial para Arduino Pro Mini con un sensor MPU6050 y cuatro motores.

## Conexiones recomendadas

- **Motores (PWM)**
  - `ena` → pin 3
  - `enb` → pin 5
  - `enc` → pin 6
  - `end` → pin 9
- **Dirección de motores**
  - `in1` → pin 10
  - `in2` → pin 11
  - `in3` → pin 12
  - `in4` → pin 13
  - `in5` → pin 2
  - `in6` → pin 7
  - `in7` → pin 8
  - `in8` → pin A1 (digital 15)
- **Sensor MPU6050** → SDA en A4 y SCL en A5
- **Potenciómetro** → A0
- **Botón de modo altura** → pin 4

## Compilación y carga

1. Abre `src.ino` en el **Arduino IDE** o utiliza `arduino-cli`.
2. Selecciona la placa **Arduino Pro Mini** (ATmega328P, 16 MHz).
3. Compila el programa y súbelo al microcontrolador.

Presiona el botón cuando el dron esté nivelado para activar el modo de
mantenimiento de altura.
