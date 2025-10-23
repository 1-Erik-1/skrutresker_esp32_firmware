#include <Adafruit_NeoPixel.h>

#define LED_PIN     8
#define NUM_LEDS    1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// User defines
typedef enum{
  idle, 
  drive,  
  armed, 
  active,
} states_t; 

uint8_t MOTOR_RIGHT_PIN = 4; 
uint8_t MOTOR_LEFT_PIN = 15;
uint8_t WEAPON_PIN = 2; 

uint16_t PWM_FREQUENCY = 24000; 
uint16_t PWM_RESOLUTION = 8; 
uint8_t PWM_CHANNEL_MOTOR_RIGHT = 0; 
uint8_t PWM_CHANNEL_MOTOR_LEFT = 1; 
uint8_t PWM_CHANNEL_WEAPON = 2; 
uint8_t PWM_INIT_DUTY = 128; 

uint8_t duty = PWM_INIT_DUTY; 
// User functions
void state_machine_run(uint8_t state) {
  switch(state){
    case idle: 

      break;

    case drive: 

      break; 

    case armed: 

      break; 

    case active: 

      break; 
  }
}

void led_setup()
{
  strip.begin();
  strip.setBrightness(5);      
  strip.show();
}

void pwm_setup()
{
  ledcAttach(MOTOR_RIGHT_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL_MOTOR_RIGHT, PWM_INIT_DUTY);
  
  ledcAttach(MOTOR_LEFT_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL_MOTOR_LEFT, PWM_INIT_DUTY);
}

void terminal_recieve()
{
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    duty = input.toInt();
    if (duty >= 0 && duty <= 255) {
      ledcWrite(PWM_CHANNEL_MOTOR_LEFT, duty);
      ledcWrite(PWM_CHANNEL_MOTOR_LEFT, duty);
      Serial.println(duty);
    } 
  }
}
void ibus_setup()
{

}

void motor_setup()
{

}

void weapon_setup()
{
  
}

// Main function
void setup() {
  pwm_setup();
  led_setup();
}

void loop() {
  terminal_recieve();
  strip.setPixelColor(0, strip.Color(0, 255, 0)); 
  delay(300);
  strip.setPixelColor(0, strip.Color(0, 0, 0)); 
}


