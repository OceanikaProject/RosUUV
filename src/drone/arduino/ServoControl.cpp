#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>


const uint8_t engines_quantity = 5;
const uint8_t pwm_pins[engines_quantity] = {3, 5, 6, 9, 10}; // 1 2 3 4 5
Servo engines[engines_quantity];

enum {
  HEAD_ST,
  LEN_ST,
  DATA_ST,
  CRC_ST
};


struct Packet
{
  char HEAD = 0x55;
  char len = 0;
  char data[8];
  char crc;
} pack;

struct PowersPacket
{
  uint16_t left_alt_power;
  uint16_t right_alt_power;
  uint16_t left_heading_power;
  uint16_t right_heading_power;
  uint16_t back_alt_power;
  uint8_t light;
  bool lock;
} powers;


int lenGet = 0;
int state = 0;
int crc = 0;
int sanity = 0;


int parseByte(char byteIn)
{
//    Serial.print("byteIn = "); Serial.println(byteIn, HEX);
    switch(state)
    {
      case HEAD_ST:
      {
        if (byteIn == 0x55)
        {
          state = LEN_ST;
//          Serial.println("HEAD_ST");
        }
      } break;


      case LEN_ST:
      {
        pack.len = byteIn;
        state = DATA_ST;
//        Serial.println("LEN_ST");
      } break;


      case DATA_ST:
      {
        if (lenGet < pack.len - 1)
        {
//          Serial.print(lenGet); Serial.print(" "); Serial.println(pack.len - 1, HEX);
          pack.data[lenGet] = byteIn;
          crc ^= byteIn;
        }
        if (lenGet == pack.len - 1)
        {
          pack.data[lenGet] = byteIn;
          state = CRC_ST;
//          Serial.println("DATA_ST");
        }
        lenGet++;
      } break;

      case CRC_ST:
      {
//        Serial.print("CRC_ST "); Serial.print(lenGet); Serial.print(" "); Serial.println(pack.len - 1, HEX);
        if (lenGet == pack.len)
        {
          lenGet = 0;
          state = 0;
          sanity = 0;
          pack.crc = byteIn;
//          Serial.print("crc = "); Serial.print(crc, HEX); Serial.print(" pack.crc = "); Serial.println(pack.crc, HEX);
          if (crc == pack.crc)
          {

            crc = 0;
            return 1;
          }
          else
          {
            crc = 0;
            return -1;
          }
        }
        else
        {
          return -1;
        }
      } break;
    }
    if (sanity++ > 19)
    {
      state = 0;
      sanity = 0;
      return -1;
    }
    return 0;
}

void printPowers()
{
  for (int i = 0; i < 8; i++)
  {
    Serial.print(pack.data[i], DEC); Serial.print(" | ");
  }
  Serial.println();
}

void setup()
{
    Serial.begin(9600);
    pinMode(11, 0);
}

void loop()
{
  if (Serial.available())
  {
    Serial.println("incomming msg");
    while (Serial.available() > 0)
    {
      int ret = parseByte(Serial.read());
      if (ret == 1)
      {
        powers.left_alt_power = map(pack.data[0], -100, 100, 1000, 2000);
	      powers.right_alt_power = map(pack.data[1], -100, 100, 1000, 2000);
	      powers.left_heading_power = map(pack.data[2], -100, 100, 1000, 2000);
        powers.right_heading_power = map(pack.data[3], -100, 100, 1000, 2000);
        powers.back_alt_power = map(pack.data[4], -100, 100, 1000, 2000);
        powers.light = pack.data[5];
        powers.lock = (bool) pack.data[6];
        printPowers();
      }

      if (!powers.lock)
      {
        for (uint8_t i = 0; i < engines_quantity; i++)
        {
          engines[i].attach(pwm_pins[i], 1000, 2000);
        }
      }
      else
      {
        for (uint8_t i = 0; i < engines_quantity; i++)
        {
          engines[i].detach();
        }
      }

      engines[0].write(powers.left_alt_power);
      engines[1].write(powers.right_alt_power);
      engines[2].write(powers.left_heading_power);
      engines[3].write(powers.right_heading_power);
      engines[4].write(powers.back_alt_power);
      // analogWrite(11, powers.light);
    

      if (powers.light == 0)
      {
        analogWrite(11, 0);
      } 
      else if (powers.light == 1)
      {
        analogWrite(11, 1000);
      }
      else if (powers.light == 2)
      {
        analogWrite(11, 1500);
      }
      else if (powers.light == 3)
      {
        analogWrite(11, 2000);
      }
    }
  }
  delay(100);
}
