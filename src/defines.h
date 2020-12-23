#include <Arduino.h>

#define LED_PIN_R 17
#define LED_PIN_G 18
#define LED_PIN_B 19

#define KEY_B_PIN 30
#define KEY_A_PIN 28
#define KEY_B !digitalRead(KEY_B_PIN)
#define KEY_A !digitalRead(KEY_A_PIN)

#define LED_R(x) digitalWrite(LED_PIN_R, !x)
#define LED_G(x) digitalWrite(LED_PIN_G, !x)
#define LED_B(x) digitalWrite(LED_PIN_B, !x)

