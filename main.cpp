#include <cstdio>
#include <mbed.h>
#include<cmath>
#include "PM2_Drivers.h"

# define M_PI 3.14159265358979323846 // number pi, an example in case you need it


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(PC_13);  // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn(); // custom functions which get executed when user button gets pressed, definition below


void spin(SpeedController& speedController, float grad){
    speedController.setDesiredSpeedRPS(1.0f*grad/abs(grad));
    thread_sleep_for(1025 * abs(grad)/360);
    speedController.setDesiredSpeedRPS(0.0f);
    thread_sleep_for(100);
}

void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    do_reset_all_once = true;
} 

// main runs as an own thread
int main()
{
    // states and actual state for state machine
    const int EXIT_STARTPOSITION = 0;
    const int GO_TO_2TE_HUERDE = 1; //move forward, after certain time measure sensor 1 and repeat 
    const int GRAB_HUERDE = 2; //rotate arm 120 degrees -> rotate head 20 degrees (or whatever)
    const int HOOK = 3; //measure sensor 2, move forward/backward
    const int DO_A_FLIP = 4; //rotate arm -120 degrees -> rotate head ... degrees -> rotate arm 40 degrees
    const int UNHOOK = 5; //move forward, rotate arm, rotate head -> rotate head, rotate arm
    const int GO_TO_END = 6; //move backward 
    const int RESET = 7; //motors to 0

    int robot_state_actual = 0;
    int wrongDistanceCounter = 0;
    int go2HuerdeCounter = 0;
    float servo1Angle = 0.0f;
    float motor1Angle = 0.0f;
    float motor2Angle = 0.0f;
    float motor3Angle = 0.0f;

    // attach button fall function to user button object, button has a pull-up resistor
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds (simple aproach to repeatedly execute main)
    const int main_task_period_ms = 500; // define main task period time in ms e.g. 50 ms -> main task runs 20 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task every main_task_period_ms


    // led on nucleo board
    DigitalOut user_led(LED1);       // create DigitalOut object to command user led

    // additional led
    DigitalOut additional_led(PB_8); // create DigitalOut object to command extra led (you need to add an aditional resistor, e.g. 220...500 Ohm)


    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // set pullup mode: sets pullup between pin and 3.3 V, so that there is a defined potential


    // Sharp GP2Y0A41SK0F, 4-40 cm IR Sensor
    float ir_distance_mV = 0.0f; // define variable to store measurement
    AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in infrared distance sensor, 0...3.3V are mapped to 0...1


    // Futaba Servo S3001 20mm 3kg Analog
    Servo servo_S1(PC_8);     // create servo objects
    
    float servo_S2_angle = 0;
    float m3_angle = 0;

    int servo_counter = 0;    // define servo counter, this is an additional variable to make the servos move
    const int loops_per_seconds = static_cast<int>(ceilf( 1.0f / (0.001f * (float)main_task_period_ms) ));


    // 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
    DigitalOut enable_motors(PB_15); // create DigitalOut object to enable dc motors
  
    FastPWM pwm_M1(PB_13); // motor M1 is used open-loop
    FastPWM pwm_M2(PA_9);  // motor M2 is closed-loop speed controlled (angle velocity)
    FastPWM pwm_M3(PA_10); // motor M3 is closed-loop position controlled (angle controlled)

    EncoderCounter  encoder_M1(PA_6, PC_7); // create encoder objects to read in the encoder counter values, since M1 is used open-loop no encoder would be needed for operation, this is just an example
    EncoderCounter  encoder_M2(PB_6, PB_7);
    EncoderCounter  encoder_M3(PA_0, PA_1);

    // create SpeedController and PositionController objects, default parametrization is for 78.125:1 gear box
    const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 100.0f;
    const float kn = 140.0f / 12.0f;
    const float k_gear = 100.0f / 100.0f;
    const float kp = 0.6f;

    SpeedController speedController_M2(counts_per_turn, kn, max_voltage, pwm_M2, encoder_M2); // default 78.125:1 gear box  with default contoller parameters
    //SpeedController speedController_M2(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M2, encoder_M2); // parameters adjusted to 100:1 gear

    PositionController positionController_M3(counts_per_turn, kn, max_voltage, pwm_M3, encoder_M3); // default 78.125:1 gear with default contoller parameters
    //PositionController positionController_M3(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_M3, encoder_M3); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
    //positionController_M3.setSpeedCntrlGain(kp * k_gear);   // adjust internal speed controller gain, this is just an example
    float max_speed_rps = 0.5f; // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
    //positionController_M3.setMaxVelocityRPS(max_speed_rps); // adjust max velocity for internal speed controller
    
    
    //float servo_S1_angle = 0.45f; // servo S1 normalized input: 0...1    
    main_task_timer.start();
    enable_motors = 1;
    thread_sleep_for(500);
    servo_S1.enable();
    positionController_M3.setDesiredRotation(0.0f);
    thread_sleep_for(1500);    
    positionController_M3.setDesiredRotation(-1.0f);
    thread_sleep_for(1500);      
    servo_S1.setNorlalisedAngle(0.108f); //60 grad
    servo1Angle = 0.108f;    

    // this loop will run forever
    while (true) {       
        main_task_timer.reset();        
        if (do_execute_main_task) {
            switch (robot_state_actual) { 
                case EXIT_STARTPOSITION: //happens twice
                    if (servo1Angle > 0.100f){ //0 grad
                        servo1Angle -= 0.001;
                        servo_S1.setNorlalisedAngle(servo1Angle);
                    }
                    else{
                        positionController_M3.setDesiredRotation(0.5f); //gerade
                        thread_sleep_for(500);
                        robot_state_actual = GO_TO_2TE_HUERDE;                      
                        spin(speedController_M2, -880.0f);
                        spin(speedController_M2, -600.0f);
                    }                                                           
                    break;    

                case GO_TO_2TE_HUERDE:                                        
                    if (servo1Angle < 0.115f){ //90 grad
                        servo1Angle += 0.001;
                        servo_S1.setNorlalisedAngle(servo1Angle);
                    }
                    else{
                        spin(speedController_M2, 437.0f);
                        positionController_M3.setDesiredRotation(2.45f); //270      
                        thread_sleep_for(1500); 
                        robot_state_actual = GRAB_HUERDE;                
                    } 
                    break;

                case GRAB_HUERDE:
                    if (servo1Angle < 0.130f){ //180 grad
                        servo1Angle += 0.001;
                        servo_S1.setNorlalisedAngle(servo1Angle);
                    }                    
                    else {
                        robot_state_actual = DO_A_FLIP;
                        thread_sleep_for(5000);
                    }                                                                                           
                    break;                            
                    
                case DO_A_FLIP:                                  
                    if (servo1Angle > 0.11f){ //0 grad
                        servo1Angle -= 0.001;
                        servo_S1.setNorlalisedAngle(servo1Angle);
                        motor3Angle = 2.45f;
                    }
                    else if(motor3Angle > 0.0f){
                        motor3Angle -= 0.01f;
                        positionController_M3.setDesiredRotation(motor3Angle); //270 grad                        
                    }
                    else{
                        robot_state_actual = UNHOOK;  
                    } 
                    break;

                case UNHOOK:                                       
                    if (servo1Angle < 0.113f){ //60 grad, M3 zu 20 grad
                        positionController_M3.setDesiredRotation(0.5);
                        servo1Angle += 0.001;
                        servo_S1.setNorlalisedAngle(servo1Angle);                        
                    }
                    else{
                        positionController_M3.setDesiredRotation(0.0f);
                        thread_sleep_for(1500);                        
                        robot_state_actual = GO_TO_END;  
                    } 
                    break;

                case GO_TO_END:
                    servo_S1.setNorlalisedAngle(0.1f);
                    spin(speedController_M2, -880.0f);
                    spin(speedController_M2, -270.0f);
                    robot_state_actual = RESET;
                    break;
                    
                case RESET:
                    servo_S1.setNorlalisedAngle(0.108f); //60 grad
                    thread_sleep_for(500);
                    positionController_M3.setDesiredRotation(0.0f);            
                    break;
                default:
                    //do nothing
                    break;
            }//end switch 

        if (do_reset_all_once) {
            do_reset_all_once = false;

            ir_distance_mV = 0.0f;
           
            pwm_M1.write(0.5f);
            
            //speedController_M2.setDesiredSpeedRPS(0.0f);
            positionController_M3.setDesiredRotation(0.0f);
            thread_sleep_for(1500);
            robot_state_actual = 0; //EXIT_STARTPOSITION
            go2HuerdeCounter = 0;            
            servo_S1.setNorlalisedAngle(0.108f); //60 grad
            thread_sleep_for(1500);
           
            additional_led = 0;
        }

        // toggling the user led
        if (servo_S1.isEnabled()){
            user_led = !user_led;
            }

        thread_sleep_for(50);
    } //end if
} //end while loop
} //end main

