
#include <Wire.h>
//#include <ams_as5048b.h>   // brown is SDA, white is SCL  ENCODER OBJECT
#include <SoftwareSerial.h>

#include <inttypes.h>
//#include <SBGC.h>    //ENCODER OBJECT
//#include <SBGC_Arduino.h>  //ENCODER OBJECT

SoftwareSerial mySerial(2, 3);  // connect (TX, RX) from gimbal controller to (pin2, pin3) respectively

//#define SBGC_CMD_CALIB_ACC 'A'
//#define SBGC_CMD_CALIB_GYRO 'g'
//#define SBGC_CMD_REALTIME_DATA_3 23
//#define SBGC_CMD_MOTOR_ON 77
#define U_DEG 3  // read encoder in degrees

const int LED = 13;
uint8_t outgoing_data[13];

// This helper function formats and sends a command to SimpleBGC Serial API
/*void SBGC_sendCommand(uint8_t cmd, void *data, uint8_t size) {
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
*/
//**************************************************************************
// This function makes sure the buffer is clear at the start and end of each loop
void clearBuffer() {
  while(mySerial.available()>0)
    mySerial.read();
}

//AMS_AS5048B encoderRoll(0x40);       // create ENCODER object for Roll
//AMS_AS5048B encoderPitch(0x41);      // create ENCODER object for Pitch
//AMS_AS5048B encoderYaw(0x42);        // create ENCODER object for Yaw
float initialRollAngle;              // initial Roll angle, used to zero out encoder
float initialPitchAngle;             // initial Pitch angle, used to zero out encoder
float initialYawAngle;               // initial Yaw angle, used to zero out encoder

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

/*void set_outgoing_data(float el_desired, float az_desired){
//  int16_t angle_pitch = SBGC_DEGREE_TO_ANGLE(el_desired);  //ENCODER OBJECT
  outgoing_data[7] = angle_pitch;  // angle pitch
  outgoing_data[8] = angle_pitch >> 8;
//  int16_t angle_yaw = SBGC_DEGREE_TO_ANGLE(az_desired);  
  outgoing_data[11] = angle_yaw;  // angle yaw
  outgoing_data[12] = angle_yaw >> 8;
}
*/
//******************* SETUP ******************* 
void setup(){
  mySerial.begin(19200);
  //SBGC_Demo_setup(&mySerial);   //ENCODER OBJECT
  Serial.begin(57600);
  pinMode(LED, OUTPUT);
  Serial.println("started");  
  
 // encoderRoll.begin();   //ENCODER OBJECT
  // encoderPitch.begin(); //ENCODER OBJECT
  //encoderYaw.begin();    //ENCODER OBJECT
  //Serial.println("started");  //Previous PrintIn given, reenable if ENCODER OBJECTS are uncommented.
  
  // calculate initial angles from encoders by taking average of 100 samples
  //double sumRoll  = 0; //Related to ENCODER
  //double sumPitch = 0;//Related to ENCODER
  //double sumYaw   = 0;//Related to ENCODER
  for(int i = 0; i < 100; i++) {
    // sumRoll  += encoderRoll.angleR(U_DEG, true);  //ENCODER OBJECT
    // sumPitch += encoderPitch.angleR(U_DEG, true);  //ENCODER OBJECT
    //sumYaw   += encoderYaw.angleR(U_DEG, true);   //ENCODER OBJECT
  }
  //initialRollAngle  = sumRoll/100.0; //Related to ENCODER
  //initialPitchAngle = sumPitch/100.0;//Related to ENCODER
  //initialYawAngle   = sumYaw/100.0;//Related to ENCODER

  //Serial.println("started");  //Not needed unless ENCODER OBJECTS reenabled
  
  //Calibrate accelerometers
  //SBGC_sendCommand(SBGC_CMD_CALIB_ACC,0,0);  //ENCODER OBJECT
  delay(6000);
  //Calibrate gyros
  //SBGC_sendCommand(SBGC_CMD_CALIB_GYRO,0,0); //ENCODER OBJECT
  delay(6000);
  //SBGC_sendCommand(SBGC_CMD_MOTOR_ON,0,0);  //ENCODER OBJECT
  delay(1000);
//  initialize_outgoing_data(30); //ENCODER OBJECT
//  set_outgoing_data(0, 0);    //ENCODER OBJECT
  //SBGC_sendCommand(67, outgoing_data, 13); //ENCODER OBJECT

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
char headerBuf[4];
char dataBuf[44]; // the data buffer is really 63 in size, but we will only read in the data we need
char checkSum[1];
double rollIMU, pitchIMU, yawIMU;
double rollENC, pitchENC, yawENC;
double convertFactor = 0.02197265625;
float yaw_angle, pitch_angle;
float az_final, el_final;

//******************* LOOP ******************* 
void loop(){
  Serial.println("looping");
  clearBuffer();
  //send read parameter command
  //SBGC_sendCommand(SBGC_CMD_REALTIME_DATA_3,0,0); // send command to read data  //ENCODER OBJECT

  // wait until we have a certain number of bytes ready
  int numBytes = 0;
  while(numBytes<(63))
  {
    numBytes = mySerial.available();
  }
  // read data into arrays
  for (int i = 0; i<4; i++)
  {
    headerBuf[i] = mySerial.read();
  }
  for (int i = 0; i<44; i++)
  {
    dataBuf[i] = mySerial.read();
  }
  // perform byte shifting
  int temp_rollIMU  = ((unsigned char)dataBuf[33] << 8) | (unsigned char)dataBuf[32];
  int temp_pitchIMU = ((unsigned char)dataBuf[35] << 8) | (unsigned char)dataBuf[34];
  int temp_yawIMU   = ((unsigned char)dataBuf[37] << 8) | (unsigned char)dataBuf[36];
 
  rollIMU  =  temp_rollIMU  * convertFactor;
  pitchIMU =  temp_pitchIMU * convertFactor;
  yawIMU   =  -temp_yawIMU   * convertFactor;
      
  // recalculate angles using ENCODER, subtract initial angle
  //rollENC  = -(encoderRoll.angleR(U_DEG, true) - initialRollAngle);
  //pitchENC = -(encoderPitch.angleR(U_DEG, true) - initialPitchAngle);
  //yawENC   = -(encoderYaw.angleR(U_DEG, true) - initialYawAngle);

  Serial.println("IMU");
  Serial.println(rollIMU);
  Serial.println(pitchIMU); 
  Serial.println(yawIMU);   
  
  //Serial.println("Encoder"); //ENCODER
  //Serial.println(rollENC);  //ENCODER
  //Serial.println(pitchENC); //ENCODER
  //Serial.println(yawENC);   //ENCODER

//This may need to be uncommented and converted for use without ENCODER OBJECT
//// reading the desired azimuth angle
////  Serial.println("desired");
//  while(!Serial.available());
//  float yaw_cmd;
//  Serial.readBytes((char*)&yaw_cmd, sizeof(yaw_cmd));
//
//  // reading the desired elevation angle
//  while(!Serial.available());
//  float pitch_cmd;
//  Serial.readBytes((char*)&pitch_cmd, sizeof(pitch_cmd));
//
//  Serial.println(yaw_cmd);
//  Serial.println(pitch_cmd);
//  
//  // move the motors to the desired position
//  set_outgoing_data(el_final, az_final);
//  SBGC_sendCommand(67, outgoing_data, 13);

  delay(1000);
}

