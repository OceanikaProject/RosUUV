#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <drone/Powers.h>

ros::NodeHandle  nh;

const uint8_t engines_quantity = 5;
const uint8_t pwm_pins[engines_quantity] = {3, 5, 6, 9, 10};
int8_t powers[engines_quantity] = {0, 0, 0, 0, 0};
Servo engines[engines_quantity];
Servo light;

struct Packet {
    uint8_t HEAD;
    uint8_t left_alt_pwm;
    uint8_t right_alt_pwm;
    uint8_t left_heading_pwm;
    uint8_t right_heading_pwm;
    uint8_t back_alt_pwm;
    uint8_t light;
    uint8_t lock;
} pack;

uint8_t prev_state = 16;

void servo_cb( const drone::Powers& cmd_msg)
{
  // powers[0] = map(cmd_msg.left_alt_pwm, -100, 100, 1148, 1832);
  // powers[1] = map(cmd_msg.right_alt_pwm, -100, 100, 1148, 1832);
  // powers[2] = map(cmd_msg.left_heading_pwm, -100, 100, 1148, 1832);
  // powers[3] = map(cmd_msg.right_heading_pwm, -100, 100, 1148, 1832);
  // powers[4] = map(cmd_msg.back_alt_pwm, -100, 100, 1148, 1832);
  // for (uint8_t i = 0; i < engines_quantity; i++)
  //   {
  //     // powers[i] = map(cmd_msg.data[i], -100, 100, 1148, 1832);
  //     engines[i].write(powers[i]);
  //   }
//   if (cmd_msg.motor_enabled != prev_state)
//   {
      if (cmd_msg.motor_enabled)
      {
        for (uint8_t i = 0; i < engines_quantity; i++)
        {
          engines[i].attach(pwm_pins[i], 1148, 1832);
        }
      }
      else
      {
        for (uint8_t i = 0; i < engines_quantity; i++)
        {
          engines[i].detach();
        }
      }
//   }

  engines[0].write(cmd_msg.left_alt_pwm);
  engines[1].write(cmd_msg.right_alt_pwm);
  engines[2].write(cmd_msg.left_heading_pwm);
  engines[3].write(cmd_msg.right_heading_pwm);
  engines[4].write(cmd_msg.back_alt_pwm);
  analogWrite(11, cmd_msg.Light_pwm);

  prev_state = cmd_msg.State;
}

ros::Subscriber<drone::Powers> sub("engines_topic", servo_cb);

void setup(){
  // for (uint8_t i = 0; i < engines_quantity; i++)
  // {
  //   engines[i].attach(pwm_pins[i], 1148, 1832);
  // }
  // light.attach(11, 1000, 2000);
  pinMode(11, 0);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{

  nh.spinOnce();
//  delay(1);
}
