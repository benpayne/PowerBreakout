// Copyright 2020 New School Mining
// Author: Ben Payne

//
// Aurdino Pro Mini 328P 3.3V 16 MHz
//  Some IDE don't have this option show put it as 5V 16MHz instead for proper baud rates.
// 

// Version 1.0 - Initial Release of board firmware.
// Version 1.1 - Add support for sending out to the group of a change of power state.  This allows 
//   groups of PSUs to turn on at the same time.

#include <SoftwareSerial.h>
#include <EEPROM.h>

// Digital Pins
#define DIP_SW_1_PIN 2
#define DIP_SW_2_PIN 3
#define DIP_SW_3_PIN 4
#define DIP_SW_4_PIN 5
#define BUTTON_PIN   8
#define PWR_PIN      7
#define RELAY_PIN    6
#define LED1_PIN     12
#define LED2_PIN     13

// RS485 Bus Pins
#define RS485_RX_PIN 11
#define RS485_TX_PIN 10
#define RS485_EN_PIN 9

#define RS485Transmit    HIGH
#define RS485Receive     LOW


// Analog pins
#define VOLTAGE_PIN  0
#define CURRENT_PIN  3  // Current boards do not support this feature

SoftwareSerial RS485Serial(RS485_RX_PIN, RS485_TX_PIN);

int led_state = 0;
unsigned long led_timer = 0;
int id = 0;
float voltage = 0;
bool power_state = false;

#define POWER_STATE_ADDR 0

// incomming commands high 4 bits are target device id
//  lower 4 bits are commend
// responses high bit indicates success 
//  low bits give length of response data that follows
#define CMD_PowerOff 1
#define CMD_PowerOn 2
#define CMD_ReadVoltage 3
#define CMD_ReadCurrent 4
#define CMD_PressButton 5
#define CMD_PressAndHoldButton 6
#define CMD_GetState 7

#define MAJOR_SW_VER 1
#define MINOR_SW_VER 1

void setup() 
{
  // If this isn't working make sure you have 16MHz selected in the arduino tools menu
  Serial.begin(56700);
  // put your setup code here, to run once:
  pinMode(DIP_SW_1_PIN, INPUT);
  pinMode(DIP_SW_2_PIN, INPUT);
  pinMode(DIP_SW_3_PIN, INPUT);
  pinMode(DIP_SW_4_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(PWR_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, RS485Receive);
  RS485Serial.begin(9600);
  //RS485Serial.listen();
  
  digitalWrite(LED1_PIN, LOW);  
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("Firmware Version: 1.1");

  digitalWrite(LED2_PIN, LOW);
  for ( int i = 0; i < MAJOR_SW_VER; i++ )
  {
    digitalWrite(LED2_PIN, HIGH);
    delay(1000);
    digitalWrite(LED2_PIN, LOW);
    delay(1000);
  }

  for ( int i = 0; i < MINOR_SW_VER; i++ )
  {
    digitalWrite(LED2_PIN, HIGH);
    delay(200);
    digitalWrite(LED2_PIN, LOW);
    delay(1000);
  }

  id = 0;
  id = id | (digitalRead(DIP_SW_1_PIN) == HIGH ? 1 : 0);
  id = id | (digitalRead(DIP_SW_2_PIN) == HIGH ? 2 : 0);
  id = id | (digitalRead(DIP_SW_3_PIN) == HIGH ? 4 : 0);
  id = id | (digitalRead(DIP_SW_4_PIN) == HIGH ? 8 : 0);
  Serial.print("ID: ");
  Serial.println(id);  

  char saved_power_state = EEPROM.read(POWER_STATE_ADDR);
  if ( saved_power_state == 1 )
  {
    digitalWrite(PWR_PIN, HIGH);
    power_state = true;
  }
  else
  {
    digitalWrite(PWR_PIN, LOW);
    power_state = false;
  }
  Serial.print("Initial Power State: ");
  Serial.println(power_state);
  // prime the LED flashing timer.
  led_timer = millis();
}


void handle_button_press()
{
  Serial.println("button press");
  power_state = !power_state;
  Serial.print("Power State: ");
  Serial.println(power_state);
  if ( power_state )
  {
    digitalWrite(PWR_PIN, HIGH);
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(RS485_EN_PIN, RS485Transmit);
    RS485Serial.write(id << 4 | CMD_PowerOn);
    digitalWrite(RS485_EN_PIN, RS485Receive);
  }
  else
  {
    digitalWrite(PWR_PIN, LOW);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(RS485_EN_PIN, RS485Transmit);
    RS485Serial.write(id << 4 | CMD_PowerOff);
    digitalWrite(RS485_EN_PIN, RS485Receive);
  }
  
  EEPROM.write(POWER_STATE_ADDR, power_state);
}

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int relayDelay = 0;

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void debounce()
{
  // read the state of the switch into a local variable:
  int reading = digitalRead(BUTTON_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        handle_button_press();
      }
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
}

void checkSerial()
{
  unsigned char data = 0;
  unsigned char response = 0;
  unsigned char res_data = 0;
  bool send_data = false;
  
  if (RS485Serial.available()) 
  {
    data = RS485Serial.read();
    Serial.print("Read Serial Command: ");
    Serial.println(data);
    
    if ( data >> 4 == id )
    {
      switch( data & 0xf )
      {
        case CMD_GetState:
          response = 0x81;
          res_data = power_state;
          send_data = true;
          break;
          
        case CMD_PowerOff:
          if( power_state == true )
          {
            power_state = false;
            digitalWrite(PWR_PIN, LOW);
            response = 0x80;
          }
          break;
          
        case CMD_PowerOn:
          if( power_state == false )
          {
            power_state = true;
            digitalWrite(PWR_PIN, HIGH);
            response = 0x80;
          }
          break;

        case CMD_ReadVoltage:
          response = 0x81;
          res_data = (unsigned char)(voltage * 10.0);
          send_data = true;
          break;
          
        case CMD_ReadCurrent:
          break;
          
        case CMD_PressButton:
          relayDelay = 1;
          digitalWrite(RELAY_PIN, HIGH);
          response = 0x80;
          break;
                    
        case CMD_PressAndHoldButton:
          relayDelay = 8;
          digitalWrite(RELAY_PIN, HIGH);
          response = 0x80;
          break;

        default:
          break;
      }

      digitalWrite(RS485_EN_PIN, RS485Transmit);
      RS485Serial.write(response);
      if ( send_data )
      {
        RS485Serial.write(res_data);
      }
      digitalWrite(RS485_EN_PIN, RS485Receive);
    }
  }
}

#define VOLT_THRESHOLD 11.5

void loop() 
{
  if ( millis()-led_timer > 1000L )
  {
    led_timer = millis();

    // update id every second.
    id = 0;
    id = id | (digitalRead(DIP_SW_1_PIN) == HIGH ? 1 : 0);
    id = id | (digitalRead(DIP_SW_2_PIN) == HIGH ? 2 : 0);
    id = id | (digitalRead(DIP_SW_3_PIN) == HIGH ? 4 : 0);
    id = id | (digitalRead(DIP_SW_4_PIN) == HIGH ? 8 : 0);

    if ( relayDelay > 0 )
      relayDelay -= 1;
    else
    {
      //Serial.println("reset relay");
      digitalWrite(RELAY_PIN, LOW);
    }

    float volts = analogRead(VOLTAGE_PIN);
    // converts reading for A/D converter to a voltage.  The board has 4:1 voltage divider to 
    //  take the 12V down to 3V.  The A/D converter reads 0-3.3V since were running at 3.3V.
    voltage = volts / 1023.0 * 3.3 * 4;

    if ( power_state == false or (power_state == true and voltage < VOLT_THRESHOLD) )
    {
      if ( led_state == 0 )
      {
        digitalWrite(LED1_PIN, HIGH);
        led_state = 1;
      }
      else
      {
        digitalWrite(LED1_PIN, LOW);
        led_state = 0;
      }

      Serial.print( "Volts: " );
      Serial.println(voltage);
    }
    else
    {
      digitalWrite(LED1_PIN, HIGH);
      // means if we flash we'll start low
      led_state = 1;
    }
  }
  
  debounce();
  checkSerial();
}
