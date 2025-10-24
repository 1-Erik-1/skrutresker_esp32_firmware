/*** User includes ***/
#include <FlyskyIBUS.h>

/*** User defines ***/
// Pinout
#define MOTOR_RIGHT_PIN (int)(4)
#define MOTOR_LEFT_PIN (int)(15)
#define WEAPON_PIN (int)(2)  // Also led pin :)
#define RADIO_PIN (int)(3)

// PWM constants
#define PWM_FREQUENCY_HZ (int)(400)
#define PWM_PERIOD_MS (float)(1000.0 / PWM_FREQUENCY_HZ)
#define PWM_RESOLUTION_BITS (int)(12)
#define PWM_INIT_DUTY (int)(128)
#define PWM_MAX_DUTY (int)(4095)
#define PWM_MAX_PULSE_WIDTH_MS (float)(2.0)
#define PWM_ZERO_PULSE_WIDTH_MS (float)(1.5)
#define PWM_MIN_PULSE_WIDTH_MS (float)(1.0)

// Watchdog constants
#define SIGNAL_TIMEOUT_MS (int)(500)

// IBUS constants
#define IBUS_CHANNELS (int)(10)

/*** User variables ***/
typedef enum {
  startup,
  idle,
  drive,
  armed,
  active,
} states_t;

typedef enum {
  right_joystick_horizontal = 1,
  right_joystick_verticle,
  left_joystick_verticle,
  left_joystick_horizontal,
  dial_A,
  dial_B,
  switch_A,
  switch_B,
  switch_C,
  switch_D
} ibus_channels_t;

FlyskyIBUS ibus(Serial2, RADIO_PIN);

int state = startup;

uint16_t ibus_buffer[IBUS_CHANNELS];

bool drive_active = false;
bool weapon_armed = false;
bool signal_timeout = false;

float weapon_throttle = 0.0;
float motor_right_throttle = 0.0;
float motor_left_throttle = 0.0;
float last_recieved_signal = 0.0;

/*** User functions ***/
/*
* 
*/
void pwm_init() {
  ledcAttach(MOTOR_RIGHT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(MOTOR_RIGHT_PIN, 0);

  ledcAttach(MOTOR_LEFT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(MOTOR_LEFT_PIN, 0);

  ledcAttach(WEAPON_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(WEAPON_PIN, 0);
}

/*
* 
*/
void pwm_update_duty(float pulsewidth_ms, uint8_t pin) {
  float dutyfloat = (pulsewidth_ms / PWM_PERIOD_MS) * PWM_MAX_DUTY;
  int duty = (int)dutyfloat;
  ledcWrite(pin, duty);
}

/*
* 
*/
void ibus_init() {
  ibus.begin();
}

/*
* 
*/
void ibus_update_data() {
  throttle_y = ibus.getChannel(right_joystick_verticle);
  throttle_x = ibus.getChannel(right_joystick_horizontal);
  weapon_throttle = ibus.getChannel(left_joystick_verticle);
  if (ibus.getChannel(switch_A) == 2000){
    weapon_armed = true;
  }
  if (ibus.getChannel(switch_D) == 2000){
    drive_active = true;
  }
}

/*
* 
*/
void motor_init() {
  pwm_update_duty(MOTOR_LEFT_PIN, PWM_ZERO_PULSE_WIDTH_MS);
  pwm_update_duty(MOTOR_RIGHT_PIN, PWM_ZERO_PULSE_WIDTH_MS);
}

/*
* 
*/
void motor_deinit() {
  pwm_update_duty(MOTOR_LEFT_PIN, 0.0);
  pwm_update_duty(MOTOR_RIGHT_PIN, 0.0);
}

/*
*
*/
void motor_update_speed_setpoint(int motor_pin, uint16_t pulse_width_ms){
  pwm_update_duty(pulse_width_ms, motor_pin);
}

/*
* 
*/
void weapon_init() {
  pwm_update_duty(WEAPON_PIN, PWM_ZERO_PULSE_WIDTH_MS);
}

/*
* 
*/
void weapon_deinit() {
  pwm_update_duty(WEAPON_PIN, 0.0);
}

/*
*
*/
void weapon_update_speed_setpoint(uint16_t pulse_width_ms){
  pwm_update_duty(pulse_width_ms, WEAPON_PIN);
}

/*
* 
*/
void terminal_init() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
}

/*
* 
*/
void reset_state_machine() {
  weapon_throttle = PWM_ZERO_PULSE_WIDTH_MS;
  weapon_armed = false;
  drive_active = false;
  state = idle;
}

/*
* 
*/
void state_machine_run() {
  switch (state) {
    case startup:
      terminal_init();
      delay(50);
      ibus_init();
      delay(50);
      pwm_init();
      delay(50);
      Serial.print("Setup complete!\n");
      state = idle;

    case idle:

      if (drive_active == true) {
        motor_init();
        state = drive;
      }
      break;

    case drive:

      if (drive_active == false) {
        motor_deinit();
        state = idle;
      } else if (weapon_armed == true) {
        weapon_init();
        state = armed;
      }
      break;

    case armed:

      if (weapon_armed == false) {
        weapon_deinit();
        state = drive;
      } else if (weapon_throttle > PWM_ZERO_PULSE_WIDTH_MS || weapon_throttle < PWM_ZERO_PULSE_WIDTH_MS) {
        state = active;
      }
      break;

    case active:
      if (weapon_throttle == PWM_ZERO_PULSE_WIDTH_MS) {
        state = armed;
      }
      break;
  }
}

/*
* 
*/
void control_loop_run() {
  if (millis() - last_recieved_signal > SIGNAL_TIMEOUT_MS) {
    signal_timeout = true;
    reset_state_machine();
  } else {
    last_recieved_signal = millis();
    signal_timeout = false;
    state_machine_run();
  }
}

/*** Main loop/setup ***/
void setup() {
  Serial.begin(115200);
  ibus_init();
}

void loop() {
  Serial.println("New_update!");
  ibus_update_data();
  delay(1000);
}
