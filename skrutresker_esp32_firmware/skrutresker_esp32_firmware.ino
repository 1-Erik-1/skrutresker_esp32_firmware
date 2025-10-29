/*** User includes ***/
#include <FlyskyIBUS.h>

/*** User defines ***/
// Pinout
#define MOTOR_RIGHT_PIN         (int)(4)
#define MOTOR_LEFT_PIN          (int)(15)
#define WEAPON_PIN              (int)(2)  // Also led pin :)
#define RADIO_PIN               (int)(3)

// PWM constants
#define PWM_FREQUENCY_HZ        (int)(400)
#define PWM_PERIOD_MS           (float)(1000.0 / PWM_FREQUENCY_HZ)
#define PWM_RESOLUTION_BITS     (int)(12)
#define PWM_INIT_DUTY           (int)(128)
#define PWM_MAX_DUTY            (int)(4095)
#define PWM_MAX_PULSE_WIDTH_MS  (float)(2.0)
#define PWM_ZERO_PULSE_WIDTH_MS (float)(1.5)
#define PWM_MIN_PULSE_WIDTH_MS  (float)(1.0)

// Timer constants
#define TIMER_INTERVAL          (float)(1000.0)

// Throttle constants
#define LOG_BASE                (float)(10.0)
#define SCALE                   (float)(1.0f / logf(LOG_BASE + 1.0f))
#define MOTOR_MAX_SPEED         (float)(0.5)

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

typedef struct {
  float forward;
  float turn;
  float right_speed;
  float left_speed;
} tank_drive_t; 

tank_drive_t tank_drive; 

FlyskyIBUS ibus(Serial2, RADIO_PIN);

uint8_t state = startup;

bool drive_active = false;
bool weapon_armed = false;
bool signal_timeout = false;

float weapon_throttle = 0.0;
float throttle_y = 0; 
float throttle_x = 0; 
float motor_right_throttle = 0.0;
float motor_left_throttle = 0.0;
float last_timer_event = 0.0;
float current_timer_event = 0.0; 

/*** User functions ***/

/**
 * @brief Initializes PWM outputs for motors and weapon.
 * 
 * Sets up LEDC PWM channels for right motor, left motor, and weapon pins
 * using the defined frequency and resolution. Initializes all outputs to 0.
 */
void pwm_init() {
  ledcAttach(MOTOR_RIGHT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(MOTOR_RIGHT_PIN, 0);

  ledcAttach(MOTOR_LEFT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(MOTOR_LEFT_PIN, 0);

  ledcAttach(WEAPON_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(WEAPON_PIN, 0);
}

/**
 * @brief Updates PWM duty cycle for a given pin.
 * 
 * @param pulsewidth_ms  Desired pulse width in milliseconds.
 * @param pin            Output pin to apply the PWM signal.
 */
void pwm_update_duty(float pulsewidth_ms, uint8_t pin) {
  float dutyfloat = (pulsewidth_ms / PWM_PERIOD_MS) * PWM_MAX_DUTY;
  int duty = (int)dutyfloat;
  ledcWrite(pin, duty);
}

/**
 * @brief Initializes the FlySky IBUS receiver.
 */
void ibus_init() {
  ibus.begin();
}

/**
 * @brief Reads and updates control inputs from the IBUS receiver.
 * 
 * Updates throttle, steering, and weapon inputs, along with
 * drive and weapon arming states. Prints diagnostic data to Serial.
 */
void ibus_update_data() {
  throttle_y = ibus.getChannel(right_joystick_verticle) / 1000.0;
  throttle_x = ibus.getChannel(right_joystick_horizontal) / 1000.0;
  weapon_throttle = ibus.getChannel(left_joystick_verticle) / 1000.0;

  weapon_armed = (ibus.getChannel(switch_A) == 2000);
  drive_active = (ibus.getChannel(switch_D) == 2000);
}

/**
 * @brief Sets motor speed based on a given pulse width.
 * 
 * @param motor_pin      Motor output pin.
 * @param pulse_width_ms Desired PWM pulse width.
 */
void motor_update_speed_setpoint(int motor_pin, float pulse_width_ms){
  pwm_update_duty(pulse_width_ms, motor_pin);
}

/**
 * @brief Implements tank drive control using joystick inputs.
 * 
 * @param throttle_x Horizontal input (turn).
 * @param throttle_y Vertical input (forward/reverse).
 * 
 * Maps joystick values into left/right motor outputs for differential drive,
 * ensuring output is clamped between valid limits.
 */
void motor_tank_drive(float throttle_x, float throttle_y)
{
  // Normalize joystick inputs from [0.0, 3.0] → [-1.0, 1.0]
  float forward = (throttle_y - 1.5f) * 2.0f;
  float turn    = (throttle_x - 1.5f) * 2.0f;

  // Logarithmicly scale throttle input
  forward = copysignf(logf(LOG_BASE * fabsf(forward) + 1.0f) * SCALE, forward);
  turn    = copysignf(logf(LOG_BASE * fabsf(turn) + 1.0f) * SCALE, turn);

  // Combine forward and turn for tank drive
  float left_speed  = forward + turn;
  float right_speed = -(forward - turn);

  // Clamp to max speed
  if (left_speed > MOTOR_MAX_SPEED) left_speed = MOTOR_MAX_SPEED;
  if (left_speed < -MOTOR_MAX_SPEED) left_speed = -MOTOR_MAX_SPEED;
  if (right_speed > MOTOR_MAX_SPEED) right_speed = MOTOR_MAX_SPEED;
  if (right_speed < -MOTOR_MAX_SPEED) right_speed = -MOTOR_MAX_SPEED;

  // Re-map back to motor range [1.0, 2.0]
  left_speed  = 1.5f + (left_speed * 0.5f);
  right_speed = 1.5f + (right_speed * 0.5f);

  // Update motors
  motor_update_speed_setpoint(MOTOR_LEFT_PIN, left_speed);
  motor_update_speed_setpoint(MOTOR_RIGHT_PIN, right_speed);
}

/**
 * @brief Updates weapon motor speed.
 * 
 * @param pulse_width_ms Desired PWM pulse width for the weapon motor.
 */
void weapon_update_speed_setpoint(float pulse_width_ms){
  // Normalize joystick inputs from [0.0, 3.0] → [-1.0, 1.0]
  float throttle = (pulse_width_ms - 1.5f) * 2.0f;
  
  // Logarithmicly scale throttle input
  throttle = copysignf(logf(LOG_BASE * fabsf(throttle) + 1.0f) * SCALE, throttle);

  // Clamp to max speed
  if (throttle > MOTOR_MAX_SPEED) throttle = MOTOR_MAX_SPEED;
  if (throttle < -MOTOR_MAX_SPEED) throttle = -MOTOR_MAX_SPEED;

  // Re-map back to motor range [1.0, 2.0]
  throttle  = 1.5f + (throttle * 0.5f);

  pwm_update_duty(throttle, WEAPON_PIN);
}

/**
 * @brief Function that executes once every second, debugging prints. 
 */
void timer_one_second(){
  current_timer_event = millis();
  if (current_timer_event - last_timer_event >= TIMER_INTERVAL){
    last_timer_event = current_timer_event; 
    Serial.printf("Uptime: %.2fs\n", millis()/1000.0);
    Serial.printf("Throttles (x, y, w): %.2f, %.2f, %.2f\n", throttle_x, throttle_y, weapon_throttle);
    Serial.printf("Motor drive speeds (l, r): %.2f, %.2f\n", tank_drive.left_speed, tank_drive.right_speed);
    Serial.printf("States (drv, arm, state, timeout): %d, %d, %d, %d\n", drive_active, weapon_armed, state, signal_timeout);
  }
}

/**
 * @brief Initializes the serial terminal for debugging.
 */
void terminal_init() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
}

/**
 * @brief Resets system state and disables all outputs.
 * 
 * Sets weapon and drive states to idle and clears throttle values.
 */
void reset_state_machine() {
  weapon_throttle = PWM_ZERO_PULSE_WIDTH_MS;
  throttle_x = PWM_ZERO_PULSE_WIDTH_MS; 
  throttle_y = PWM_ZERO_PULSE_WIDTH_MS; 
  weapon_armed = false;
  drive_active = false;
  state = idle;
  state_machine_run();
}

/**
 * @brief Executes the main robot state machine.
 * 
 * Handles startup, idle, driving, and armed weapon states.
 * Transitions occur based on IBUS switch inputs and control flags.
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
      break;

    case idle:
      if (drive_active == true) {
        state = drive;
      }
      break;

    case drive:
      motor_tank_drive(throttle_x, throttle_y);

      if (drive_active == false) {
        motor_update_speed_setpoint(MOTOR_LEFT_PIN, 0.0);
        motor_update_speed_setpoint(MOTOR_RIGHT_PIN, 0.0);
        state = idle;
      } else if (weapon_armed == true) {
        state = armed;
      }
      break;

    case armed:
      motor_tank_drive(throttle_x, throttle_y);
      weapon_update_speed_setpoint(PWM_ZERO_PULSE_WIDTH_MS);

      if (weapon_armed == false || drive_active == false) {
        weapon_update_speed_setpoint(0.0);
        state = drive;
      } else if (weapon_throttle < 1.4 || weapon_throttle > 1.6) {
        state = active; 
      }
      break;

    case active: 
      motor_tank_drive(throttle_x, throttle_y);
      weapon_update_speed_setpoint(weapon_throttle);

      if ((weapon_throttle >= 1.4 && weapon_throttle <= 1.6) || weapon_armed == false || drive_active == false){
        weapon_update_speed_setpoint(PWM_ZERO_PULSE_WIDTH_MS);
        state = armed; 
      }
      break;
  }
}

/**
 * @brief Runs the main control loop logic.
 * 
 * Checks for lost signal timeout and updates
 * input data, then runs the state machine.
 */
void control_loop_run() {
  if (state != startup && ibus.hasFailsafe() == true){
    signal_timeout = true; 
    motor_update_speed_setpoint(MOTOR_LEFT_PIN, 0.0);
    motor_update_speed_setpoint(MOTOR_RIGHT_PIN, 0.0);
    motor_update_speed_setpoint(WEAPON_PIN, 0.0);
    reset_state_machine(); 
    delay(50);
  }else{
    signal_timeout = false; 
    if (state != startup){ibus_update_data();}
    state_machine_run();
  }
  //timer_one_second();
}

/*** Main loop/setup ***/
void setup() {

}

void loop() {
  control_loop_run();
}
