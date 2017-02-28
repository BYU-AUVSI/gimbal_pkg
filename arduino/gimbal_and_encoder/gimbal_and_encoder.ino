 /*
This file was adapted from Jae's gimbal control code. The original code is meant to be used with encoders. 
Lots of the lines commented out are related to these encoders that we are not currently using.
The main loop waits for data to be available from the serial port (on pins 2,3 of the arduino). It reads in the 
available data as the commanded pitch angle and writes this commanded angle to the BGC board.

Communication with BGC is serial "mySerial"
Communication between arduino and computer is serial "Serial"
Communication with encoder is over SPI
*/

#include <Wire.h>
#include <SoftwareSerial.h>

#include <inttypes.h>
#include <SBGC.h> //Header for Gimbal Controller Functions
#include <SBGC_Arduino.h> //Header for Gimbal Controller Functions

#include <SPI.h> //SPI Interface for communication with encoder

SoftwareSerial mySerial(2, 3);  // connect (TX, RX) from gimbal controller to (pin2, pin3) respectively

#define CS 10 //Chip or Slave select for SPI

//BGC Params
#define SBGC_CMD_CALIB_ACC 'A'
#define SBGC_CMD_CALIB_GYRO 'g'
#define SBGC_CMD_REALTIME_DATA_3 23
#define SBGC_CMD_MOTOR_ON 77
#define U_DEG 3   

const int LED = 13;
uint8_t outgoing_data[13];

// Encoder Variables
uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];
float deg = 0.00;

// This helper function formats and sends a command to SimpleBGC Serial API
void SBGC_sendCommand(uint8_t cmd, void *data, uint8_t size) {
  uint8_t i, checksum=0;
  // Header
  mySerial.write('>');
  mySerial.write(cmd);
  mySerial.write(size);
  mySerial.write(cmd+size);
  // Body
  for(i=0;i<size;i++) {
    checksum+= ((uint8_t*)data)[i];
    mySerial.write(((uint8_t*)data)[i]);
  }
  mySerial.write(checksum);
} 

//**************************************************************************
// This function makes sure the buffer is clear at the start and end of each loop
void clearBuffer() {
  while(mySerial.available()>0)
    mySerial.read();
}

void initialize_outgoing_data(int motor_speed){
  outgoing_data[0] = 2;   // control mode
  outgoing_data[1] = 0;  // speed roll
  outgoing_data[2] = 0;
  outgoing_data[3] = 0;  // angle roll
  outgoing_data[4] = 0;
  int16_t speed_pitch = motor_speed; // * SBGC_SPEE7D_SCALE; ENCODER OBJECT
  outgoing_data[5] = speed_pitch;  // speed pitch
  outgoing_data[6] = speed_pitch >> 8;
  outgoing_data[7] = 0;  // angle pitch
  outgoing_data[8] = 0;
  int16_t speed_yaw = motor_speed; //* SBGC_SPEED_SCALE;  ENCODER OBJECT
  outgoing_data[9] = speed_yaw;  // speed yaw
  outgoing_data[10] = speed_yaw >> 8;
  outgoing_data[11] = 0;  // angle yaw
  outgoing_data[12] = 0;  
}

void set_outgoing_data(float el_desired){
  int16_t angle_pitch = SBGC_DEGREE_TO_ANGLE(el_desired);  //ENCODER OBJECT
  outgoing_data[7] = angle_pitch;  // angle pitch
  outgoing_data[8] = angle_pitch >> 8;

  //Serial.println("outgoing data");
  //Serial.println(angle_pitch);
  //Serial.println("el_desired");
  //Serial.println(el_desired);
}

uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS,LOW);     //select spi device
   msg_temp = SPI.transfer(msg);    //send and recieve
   digitalWrite(CS,HIGH);    //deselect spi device
   return(msg_temp);      //return recieved byte
}

//******************* SETUP ******************* 
void setup(){
  
  // BGC Setup
  mySerial.begin(19200);
  SBGC_Demo_setup(&mySerial);
  
  // Encoder Setup
  pinMode(CS,OUTPUT);//Slave Select
  digitalWrite(CS,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  
  // Serial for coms with computer
  Serial.begin(57600);
  pinMode(LED, OUTPUT);
  //Serial.println("started");  
  Serial.flush();
  delay(2000);
  SPI.end();
  
  //Calibrate accelerometers
  //SBGC_sendCommand(SBGC_CMD_CALIB_ACC,0,0);  //BGC 
//  delay(6000);
  //Calibrate gyros
  //SBGC_sendCommand(SBGC_CMD_CALIB_GYRO,0,0); //BGC
//  delay(6000);
  //SBGC_sendCommand(SBGC_CMD_MOTOR_ON,0,0);  //BGC
  
  delay(1000);
  
  float step_speed = 1000; //This number relates to the speed of response for the commanded gimbal angle (higher # = faster response)
  
  initialize_outgoing_data(step_speed); // Input = MOTOR_SPEED for speed of response to input
  set_outgoing_data(-90);    //ENCODER OBJECT
  SBGC_sendCommand(67, outgoing_data, 13); //ENCODER OBJECT

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}

//************************************************************//
// Define Gimbal Parameters for main loop
char checkSum[1];

//******************* LOOP ******************* 
void loop(){
  //Serial.println("looping");
  clearBuffer();
  
  // Encoder SPI Stuff
   uint8_t recieved = 0xA5;    //just a temp vairable
   ABSposition = 0;    //reset position vairable
   
   SPI.begin();    //start transmition
   digitalWrite(CS,LOW);
   
   SPI_T(0x10);   //issue read command
   
   recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send
   
   while (recieved != 0x10)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x00);    //cleck again if encoder is still working 
     delay(2);    //wait a bit
   }
   
   temp[0] = SPI_T(0x00);    //Recieve MSB
   temp[1] = SPI_T(0x00);    // recieve LSB
   
   digitalWrite(CS,HIGH);  //just to make sure   
   SPI.end();    //end transmition
   
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
   if (1)//(ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
   {
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
     Serial.println(deg);     //send position in degrees
   }   
   // END ENCODER SPI STUFF

  // GIMBAL ANGLE COMMAND STUFF
 // reading the desired elevation angle
 if (Serial.available()){
  float pitch_cmd;
  Serial.readBytes((char*)&pitch_cmd, sizeof(pitch_cmd)); 
  pitch_cmd = -pitch_cmd;
  set_outgoing_data(pitch_cmd);
  SBGC_sendCommand(67, outgoing_data, 13);
 }

  delay(100);
}

