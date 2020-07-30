// Copyright 2020 New School Mining
// Author: Ben Payne

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
#define CURRENT_PIN  3

SoftwareSerial RS485Serial(RS485_RX_PIN, RS485_TX_PIN);

int led_state = 0;
unsigned long led_timer = 0;
int id = 0;
float voltage = 0;
bool power_state = false;

#define POWER_STATE_ADDR 0

void setup() {
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
  
  led_timer = millis();
  digitalWrite(LED1_PIN, LOW);  
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("Firmware Version: 1.0");
  
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
  }
  else
  {
    digitalWrite(PWR_PIN, LOW);
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

// incomming commands high 4 bits are target device id
//  lower 4 bits are commend
// responses high bit indicates success 
//  low bits give length of response data that follows
#define CMD_PowerOff 0
#define CMD_PowerOn 1
#define CMD_ReadVoltage 2
#define CMD_ReadCurrent 3
#define CMD_PressButton 4
#define CMD_PressAndHoldButton 5
#define CMD_GetState 6

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

void loop() {
  if ( millis()-led_timer > 1000L )
  {
    led_timer = millis();

    if ( relayDelay > 0 )
      relayDelay -= 1;
    else
    {
      //Serial.println("reset relay");
      digitalWrite(RELAY_PIN, LOW);
    }
    
    if ( led_state == 0 )
    {
      float volts = analogRead(VOLTAGE_PIN);
      voltage = volts / 1023.0 * 3.3 * 4;

      // LED will flash if volatge less than threshold, otherwise just keep checking every second.  
      if ( voltage < VOLT_THRESHOLD )
      {
        digitalWrite(LED1_PIN, HIGH);
        led_state = 1;
      }

      Serial.print( "Volts: " );
      Serial.println(voltage);
  
      //float amps = analogRead(CURRENT_PIN);
      //amps = amps / 1023.0 * 150;
      //Serial.print( "Amps: " );
      //Serial.println(amps);
    }
    else
    {
      digitalWrite(LED1_PIN, LOW);
      led_state = 0;
    }
  }
  
  debounce();
  checkSerial();
}
