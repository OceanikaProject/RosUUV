#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>


const uint8_t engines_quantity = 5;
const uint8_t pwm_pins[engines_quantity] = {3, 5, 6, 9, 10};
int8_t powers[engines_quantity] = {0, 0, 0, 0, 0};
Servo engines[engines_quantity];
Servo light;


struct Packet {
    uint8_t HEAD = 0x55;
    uint8_t left_alt_pwm;
    uint8_t right_alt_pwm;
    uint8_t left_heading_pwm;
    uint8_t right_heading_pwm;
    uint8_t back_alt_pwm;
    uint8_t light;
    uint8_t lock;
} pack;


uint8_t prev_state = 16;

uint8_t input[16];

void parse(uint8_t *arr)
{
    if (arr[0] == pack.HEAD)
    {
        pack.left_alt_pwm = arr[1] << 8 | arr[2];
        pack.right_alt_pwm = arr[3] << 8 | arr[4];
        pack.left_heading_pwm = arr[5] << 8 | arr[6];
        pack.right_heading_pwm = arr[7] << 8 | arr[8];
        pack.back_alt_pwm = arr[9] << 8 | arr[10];
        pack.light = arr[10];
        pack.lock = arr[11];
        Serial.print(pack.left_alt_pwm); Serial.print('\t'); Serial.print(pack.right_alt_pwm); Serial.print('\t');
        Serial.print(pack.left_heading_pwm); Serial.print('\t'); Serial.print(pack.right_heading_pwm); Serial.print('\t');
        Serial.print(pack.back_alt_pwm); Serial.print('\t'); Serial.println(pack.lock);
    }
}

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available())
    {
        uint8_t i = 0;
        while (Serial.available())
        {
            input[i] = Serial.read();
        }
        parse(input);
    }
}