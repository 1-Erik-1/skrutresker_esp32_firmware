/*** User defines ***/ 
// Pinout 
#define MOTOR_RIGHT_PIN (int)(4)
#define MOTOR_LEFT_PIN  (int)(15)
#define WEAPON_PIN      (int)(2) // Also led pin :)

// PWM constants
#define PWM_FREQUENCY_HZ (int)(400)
#define PWM_PERIOD_MS (float)(1000.0/PWM_FREQUENCY_HZ)
#define PWM_RESOLUTION_BITS (int)(12)
#define PWM_INIT_DUTY  (int)(128)
#define PWM_MAX_DUTY (int)(4095)
#define PWM_MAX_PULSE_WIDTH_MS (float)(2.0)
#define PWM_ZERO_PULSE_WIDTH_MS (float)(1.5)
#define PWM_MIN_PULSE_WIDTH_MS (float)(1.0)

// Watchdog constants 
#define SIGNAL_TIMEOUT_MS (int)(500)

/*** User variables ***/ 
typedef enum{
  startup,
  idle, 
  drive,  
  armed, 
  active,
} states_t; 

int state = startup; 

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
void pwm_init()
{
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
void pwm_update_duty(float pulsewidth_ms, uint8_t pin)
{
  float dutyfloat = (pulsewidth_ms / PWM_PERIOD_MS) * PWM_MAX_DUTY;
  int duty = (int)dutyfloat;
  ledcWrite(pin, duty);
}

/*
* 
*/
void terminal_receive() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    float pulsewidth_ms = input.toFloat(); 

    if (pulsewidth_ms >= PWM_MIN_PULSE_WIDTH_MS && pulsewidth_ms <= PWM_MAX_PULSE_WIDTH_MS){
      pwm_update_duty(pulsewidth_ms, MOTOR_LEFT_PIN);
      pwm_update_duty(pulsewidth_ms, MOTOR_RIGHT_PIN);
      pwm_update_duty(pulsewidth_ms, WEAPON_PIN);
    }   
  } 
}

/*
* 
*/
void ibus_init()
{

}

/*
* 
*/
void ibus_update_data()
{

}

/*
* 
*/
void motor_init()
{
  pwm_update_duty(MOTOR_LEFT_PIN, PWM_ZERO_PULSE_WIDTH_MS);
  pwm_update_duty(MOTOR_RIGHT_PIN, PWM_ZERO_PULSE_WIDTH_MS);
}

/*
* 
*/
void motor_deinit()
{
  pwm_update_duty(MOTOR_LEFT_PIN, 0.0);
  pwm_update_duty(MOTOR_RIGHT_PIN, 0.0);
}

/*
* 
*/
void weapon_init()
{
  pwm_update_duty(WEAPON_PIN, PWM_ZERO_PULSE_WIDTH_MS);
}

/*
* 
*/
void weapon_deinit()
{
  pwm_update_duty(WEAPON_PIN, 0.0);
}

/*
* 
*/
void terminal_init()
{
  Serial.begin(115200);              
  while (!Serial) { delay(10); }      
}

/*
* 
*/
void reset_state_machine()
{
  weapon_throttle = PWM_ZERO_PULSE_WIDTH_MS; 
  weapon_armed = false; 
  drive_active = false; 
  state = idle; 
}

/*
* 
*/
void state_machine_run() {
  switch(state){
    case startup: 
      terminal_init();
      ibus_init();
      pwm_init();
      Serial.print("Setup complete!\n");
      state = idle; 

    case idle: 

      if (drive_active == true){
        motor_init();
        state = drive; 
      }
      break;

    case drive: 

      if (drive_active == false){
        motor_deinit();
        state = idle; 
      }else if(weapon_armed == true){
        weapon_init();
        state = armed; 
      }
      break; 

    case armed: 

      if (weapon_armed == false){
        weapon_deinit();
        state = drive; 
      }else if(weapon_throttle > PWM_ZERO_PULSE_WIDTH_MS || weapon_throttle < PWM_ZERO_PULSE_WIDTH_MS){
        state = active; 
      }
      break; 

    case active: 
      if (weapon_throttle == PWM_ZERO_PULSE_WIDTH_MS){
        state = armed; 
      }
      break; 
  }
}

/*
* 
*/
void control_loop_run()
{
  if (millis() - last_recieved_signal > SIGNAL_TIMEOUT_MS){
  signal_timeout = true; 
  reset_state_machine();
  }else{
    last_recieved_signal = millis();
    signal_timeout = false; 
    state_machine_run();
  }
}

/*** Main loop/setup ***/
void setup() {

}

void loop() {
  control_loop_run();
}


