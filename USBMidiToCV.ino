
/* 
 * Converter from USB/MIDI output from Casio CTK-810 to CV for Analog Synthesizer.
 *
 * Also works for MIDITECH Midi i2 Control MIDI controller.
 *
 * Includes joystick controller to modify CV output.
 *
 * Note range from CTK-810:
 * Lowest note  is 0x24
 * Highest note is 0x60
 *
 * Uses the:
 * USB-MIDI class driver for USB Host Shield 2.0 Library
 * Copyright 2012 Yuuichi Akagawa
 * https://github.com/YuuichiAkagawa/USBH_MIDI
 *
 */


#include <SPI.h>
#include <Usb.h>
#include <usbh_midi.h>

#include <DAC.h>

#include <MIDI.h>

//#define CASIO 1
#define MIDITECH 1

#define CASIO_VID 0x07CF
#define CASIO_PID 0x6802
#define MIDITECH_VID 0x1ACC
#define MIDITECH_PID 0x1A0D

#define CONNECT_FLASH_PERIOD 500 // flash connect LED every 1/2 second until we are connected.

// Pins for DAC:
#define LDAC_PIN 5
#define SHDN_PIN 4
#define SS_PIN   6
#define GATE_PIN 3
#define NOTE_ON_LED_PIN 2
#define CONNECT_LED_PIN A5

#define LOWEST_NOTE  0x24

#define NOTE_OFF   0x80 
#define NOTE_ON    0x90
#define PITCH_BEND 0xE0
#define MOD_WHEEL  0xB0

USB  Usb;

int notes_on=0;


#ifdef CASIO
MIDI  Midi(&Usb, CASIO_PID, CASIO_VID);
#endif

#ifdef MIDITECH
MIDI  Midi(&Usb, MIDITECH_PID, MIDITECH_VID);
#endif

DAC dac( LDAC_PIN, SHDN_PIN, SS_PIN );

const float SEMITONE_INC = 1.0/12.0;  // semitone increment

#define HORIZONTAL_PIN 0
#define VERTICAL_PIN 1

int last_horizontal_val = 0;
int last_vertical_val = 0;

float note_voltage = 0.0;
float mod_voltage = 0.0;


long connect_millis;
boolean connect_led_on;


void setup()
{


  Serial.begin(9600);

  //Workaround for non UHS2.0 Shield 
  //pinMode(7,OUTPUT);
  //digitalWrite(7,HIGH);

  if (Usb.Init() == -1) {
    Serial.println("USB Initialization Failed");
    while(1); //halt
  }//if (Usb.Init() == -1...
  Serial.println("USB Initialized OK");
  delay( 200 );

  // Set the Gate Pin as output and turn it off:
  pinMode( GATE_PIN, OUTPUT );
  digitalWrite( GATE_PIN, LOW );

  pinMode( NOTE_ON_LED_PIN, OUTPUT );
  digitalWrite( NOTE_ON_LED_PIN, LOW );

  pinMode( CONNECT_LED_PIN, OUTPUT );
  digitalWrite( CONNECT_LED_PIN, LOW );
  connect_millis = millis();

}



void loop()
{
  int horizontal_val = 0;
  int vertical_val = 0;


  Usb.Task();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    MIDI_poll();

    // Connected LED is steady when we are actually running:
    connect_led_on = true;

  } 
  else {
    if( (millis() - connect_millis) >= CONNECT_FLASH_PERIOD ) {
      // if we are not yet connected then check the time and see if it's time to flash the LED      
      if( connect_led_on ) {
        connect_led_on = false;
      } 
      else {
        connect_led_on = true; 
      }

      connect_millis = millis();
    }
  }

  if( connect_led_on ) {
    digitalWrite( CONNECT_LED_PIN, HIGH );
  } 
  else {
    digitalWrite( CONNECT_LED_PIN, LOW );
  }

  horizontal_val = analogRead( HORIZONTAL_PIN );
  if( abs(horizontal_val - last_horizontal_val) > 5 ) {
    Serial.print("Horizontal val = ");
    Serial.println(horizontal_val, DEC );
    last_horizontal_val = horizontal_val;
    // Convert this to a voltage offset. The maximum offset is two whole semi-tone added or removed from the note.
    float voltage_offset = ((float )(horizontal_val-512) /1024)* SEMITONE_INC * 2;
    float voltage = note_voltage - voltage_offset;
    if( voltage < 0.0 )
      voltage = 0.0;
    if( voltage > 5.0 )
      voltage = 5.0;
    dac.setVoltage( voltage, CHANNEL_A );
    Serial.print("Pitch voltage = ");
    Serial.println(voltage, DEC );
  }


  vertical_val = analogRead( VERTICAL_PIN );  
  if( abs(vertical_val - last_vertical_val) > 5 ) {
    Serial.print("Vertical val = ");
    Serial.println(vertical_val, DEC );

    mod_voltage = abs(vertical_val - 512 )/512.0 * 5.0;
    dac.setVoltage( mod_voltage, CHANNEL_B );
    Serial.print("Mod voltage = ");
    Serial.println(mod_voltage, DEC );
    last_vertical_val = vertical_val;
  }

  delay(1);
}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{
  byte outBuf[ 3 ];
  if( Midi.RcvData(outBuf) == true ){
    //MIDI Output
    Serial.print("Got MIDI Data: " );
    Serial.print(outBuf[0], HEX );
    Serial.print(" " );
    Serial.print(outBuf[1], HEX );
    Serial.print(" " );
    Serial.println(outBuf[2], HEX );

    // Examine outBuf[0] to find Note On/Note Off events:
    if( outBuf[0] == NOTE_ON ) {
      note_on( outBuf[1] );       
    } 
    else if( outBuf[0] == NOTE_OFF ) {
      note_off( outBuf[1] ); 
    }
    else if( outBuf[0] == PITCH_BEND ) {
      pitch_bend( outBuf[2] ); 
    }
    else if( outBuf[0] == MOD_WHEEL and outBuf[1] == 1 ) {
      mod_wheel( outBuf[2] ); 
    }
    else if( outBuf[0] == MOD_WHEEL and outBuf[1] == 0x4A ) {
      glide( outBuf[2] ); 
    }
  }

}

// Handle a note on event. Set the voltage and turn the GATE on:
void note_on( byte note)
{

  note_voltage = (float)(note - (byte)LOWEST_NOTE ) * SEMITONE_INC;  
  if( note_voltage < 0.0 ) {
    note_voltage = 0.0; 
  }
  if( note_voltage > 5.0 ) {
    note_voltage = 5.0; 
  }

  Serial.print("Note = " );
  Serial.println(note, HEX );

  Serial.print("Pitch voltage = ");
  Serial.println(note_voltage, DEC );
  dac.setVoltage( note_voltage, CHANNEL_A );

  if( notes_on  > 0 ){
    // Turn the gate off shortly to retrigger the envelope:
    digitalWrite( GATE_PIN, LOW);
    delay(10); 
  }


  digitalWrite( GATE_PIN, HIGH );
  digitalWrite( NOTE_ON_LED_PIN, HIGH );

  notes_on += 1;


}

// Handle a note-off event. Simply turn the gate off if we have no more notes on:
void note_off( byte note ) {

  notes_on -= 1;
  if( notes_on < 0 ) {
    notes_on = 0;
  }
  if( notes_on == 0 ) {
    digitalWrite( GATE_PIN, LOW ); 
    digitalWrite( NOTE_ON_LED_PIN, LOW );
  }

}

void pitch_bend( byte bend ) {


  float voltage_offset = float(bend);
  voltage_offset = ((61.0 - voltage_offset)/127.0 ) * SEMITONE_INC * 2;
  Serial.print("Bend offset " );
  Serial.println(voltage_offset, DEC );

  dac.setVoltage( note_voltage-voltage_offset, CHANNEL_A );

}


void mod_wheel( byte bend ) {

  float mod_voltage = float(bend);
  mod_voltage = mod_voltage/127.0 * 5.0;
  Serial.print("Mod voltage " );
  Serial.println(mod_voltage, DEC );

  dac.setVoltage( mod_voltage, CHANNEL_B );

}

void glide( byte glide_val ) {

  Serial.print("Glide val " );
  Serial.println(glide_val, DEC );

}








