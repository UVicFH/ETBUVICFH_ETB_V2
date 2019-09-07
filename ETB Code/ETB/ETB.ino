// Include library to talk to CAN shield
#include <SPI.h>
#include "mcp_can.h"
#include "ETB.h"
#include "AS5601.h"
AS5601 Sensor;
// Define the CAN select pin and ID mask
#define SPI_CS_PIN 9

// Length of received message
byte len = 1;
long last_recv = 0;

// Create a variable to store the receieved message
byte throttle_input = 0;

//values to be sent over CAN
uint16_t currentDraw_mA = 0;                  
uint16_t voltage_mV = 0;                    
uint8_t temp_degCx2 = 0;              
uint16_t zeroedHallPosition_percentx10 = 0;
int Throt = 0;
//sensor feedback not sent over CAN
uint16_t absoluteHallPosition_nominal = 0;  
int32_t zeroedHallPosition_nominal = 0;         //note this is signed
uint16_t hallZeroPosition_nominal = 0;
float zeroedHallPosition_percent = 0;

// Initialize with the CAN Shield select pin
MCP_CAN CAN(SPI_CS_PIN);

uint16_t as5048aReadAndClearError(void)
{
  uint16_t spiSendCommand = HALL_GET_ERROR;
  uint16_t hallErrorMsg = 0b1111111111111111;

  spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
  spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

  SPI.beginTransaction(SPI_SETTINGS_HALL);

  digitalWrite(SPI_HALL_CS, LOW);
  SPI.transfer16(spiSendCommand);         // returned value is garbage angle from last request
  digitalWrite(SPI_HALL_CS, HIGH);

  spiSendCommand = HALL_GET_ANGLE;        // reset spi command to read angle (in preparation for next request)
  spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
  spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

  digitalWrite(SPI_HALL_CS, LOW);
  hallErrorMsg = SPI.transfer16(spiSendCommand);  // returned value is error message from last request
  digitalWrite(SPI_HALL_CS, HIGH);

  SPI.endTransaction();
  return hallErrorMsg;
}

//prints the value of the error register
void as5048aCheckError(uint16_t command) {
  if ((command & 0b0100000000000000) == 0b0100000000000000) 
  {
    Serial.print("Hall Sensor Error: ");
    Serial.println(as5048aReadAndClearError(), BIN);
  }
  return; 
}

//removes the first two bits of any received message, checks for error bit
uint16_t as5048aRemoveParity(uint16_t command)
{
  as5048aCheckError(command);
  return (command &= 0b0011111111111111);                 //clears parity and error bit
}


//returns the parity bit to create even parity in a 16 bit message
uint8_t as5048aSetParity(uint16_t value){
  uint8_t cnt = 0;
  uint8_t i;

  for (i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

//sends the request to the hall sensor to read the position. calculates the read value from the sensor.
void as5048aReadAndAverage(uint16_t spiSendCommand)
{
  uint32_t hallPositionSum_nominal = 0;

  spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
  spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

  SPI.beginTransaction(SPI_SETTINGS_HALL);
  for(int i=0; i<HALL_AVERAGE_SIZE; i++)
  {
    digitalWrite(SPI_HALL_CS, LOW);
    hallPositionSum_nominal += (uint32_t) (16384 - as5048aRemoveParity(SPI.transfer16(spiSendCommand)));
    digitalWrite(SPI_HALL_CS, HIGH);
    delayMicroseconds(250);
  } 
  SPI.endTransaction();

  absoluteHallPosition_nominal = (uint16_t) (hallPositionSum_nominal >> filterShiftSize(HALL_AVERAGE_SIZE));
}

//used to subtract the read value from the pre-established zero point. Note integer types used for safe execution
void getZeroedHallPosition(void)
{
  as5048aReadAndAverage(HALL_GET_ANGLE);
  zeroedHallPosition_nominal =  (int32_t) absoluteHallPosition_nominal - (int32_t) hallZeroPosition_nominal;
  if(zeroedHallPosition_nominal < 0)
  {
    zeroedHallPosition_nominal = 0;
  }
  zeroedHallPosition_percentx10 = (((uint32_t) zeroedHallPosition_nominal)*10) >> 5; //same as dividing by 32, or doing zeroedHallPosition_nominal*360.0/16384.0*100.0/69.2.
}


//used at the beginning of program execution to find the zero point of the sensor.
void findHallZeroPosition(void)
{
  uint32_t hallZeroPositionSum_nominal = 0;

  as5048aReadAndAverage(HALL_GET_ANGLE);
  hallZeroPosition_nominal = absoluteHallPosition_nominal;
  
  Serial.print("hallZeroPosition_nominal = ");
  Serial.println(hallZeroPosition_nominal);
}

//sends the outgoing CAN message with updated variables
void sendCanMsg(void)
{
  uint8_t canSendBuffer1[8];
  uint8_t canSendBuffer2[8];

  canSendBuffer1[0] = zeroedHallPosition_percentx10 & 0b11111111;
  canSendBuffer1[1] = zeroedHallPosition_percentx10 >> 8;
  canSendBuffer1[2] = currentDraw_mA & 0b11111111;
  canSendBuffer1[3] = currentDraw_mA >> 8;
  canSendBuffer1[4] = voltage_mV & 0b11111111;
  canSendBuffer1[5] = voltage_mV >> 8;
  canSendBuffer1[6] = temp_degCx2;
  canSendBuffer1[7] = 0;

  canSendBuffer2[0] = (int8_t) throttle_input;
  canSendBuffer2[1] = 0; //((int8_t) (100.0 * controllerResult)) & 0b01111111;
  canSendBuffer2[2] = 0;
  canSendBuffer2[3] = 0;
  canSendBuffer2[4] = 0;
  canSendBuffer2[5] = 0;
  canSendBuffer2[6] = 0; 
  canSendBuffer2[7] = 0; 
  
  SPI.beginTransaction(SPI_SETTINGS_CAN);
  CAN.sendMsgBuf(CAN_FEEDBACK_MSG_ADDRESS,0,8,canSendBuffer1);
  CAN.sendMsgBuf(0x104, 0, 8, canSendBuffer2);
  SPI.endTransaction();
}

//gets the CAN message in the buffer, and reads the required values
void receiveCanMsg(void)
{
  // If the bus is live recieve the throttle_input
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    // Read the throttle_input and print it via serial
    CAN.readMsgBuf(&len, &throttle_input);
    
    last_recv = millis();
  }
}


//calculates the current consumption of the device. only reports current value; no averaging.
void calculateCurrent(void)
{
  currentDraw_mA = (uint16_t) (analogRead(CURRENT_SENS) * 10);  // analogRead(CURRENT_SENS)/1023.0*5/(0.01*0.05*1000)*1000
}

//calculates the voltage input to the device, accounts for voltage drop through shunt
void calculateVoltage(void)
{
  voltage_mV = analogRead(VOLTAGE_SENS) * 19.68 + 0.05 * currentDraw_mA;
}

//calculates temperature according to thermistor calibrated curve
void calculateTemperature(void)
{
  float rThermistor_ohms = (float) 5.0*5000.0/(analogRead(TEMP_SENS)*5.0/1023.0);
    float lnOperand = rThermistor_ohms/(10000*pow(2.71828, (-THERMISTOR_B_CONSTANT/298.0)));
    temp_degCx2 = (uint8_t) (2.0 * (THERMISTOR_B_CONSTANT / log(lnOperand) - 273.0 + THERMISTOR_CALIB_DEGC));
}

//calculates the number of bit shifts required to perform a given division
uint8_t filterShiftSize(uint8_t filterSize)
{
  uint8_t shiftSize = 0;
  switch(filterSize)
  {
    case 1:
      shiftSize = 0;
      break;
    case 2:
      shiftSize = 1;
      break;
    case 4:
      shiftSize = 2;
      break;
    case 8:
      shiftSize = 3;
      break;
    case 16:
      shiftSize = 4;
      break;
    case 32:
      shiftSize = 5;
      break;
    case 64:
      shiftSize = 6;
      break;
    case 128:
      shiftSize = 7;
      break;
    default:
      shiftSize=0;
      break;

  }
  return shiftSize; 
}

void setup()
{
  // Start the serial stream
  Serial.begin(115200);
  SPI.begin();  
//  // Initialize the CAN Bus and check for errors
//  while (CAN_OK != CAN.begin(CAN_500KBPS))
//  {
//      Serial.println("CAN BUS Shield init fail");
//      Serial.println("Init CAN BUS Shield again");
//      delay(100);
//  }
//  
//  Serial.println("CAN BUS Shield init ok!");
//
//  //Set can masks and filters to only accept the throttle request message (this is critical so that the SPI bus doesn't get bogged down
//  // with traffic due to the MCP2515 receiving other messages on the bus)
//  CAN.init_Mask(0, 0, 0xFFF);                       //must set both masks; use standard CAN frame
//  CAN.init_Mask(1, 0, 0xFFF);                       //must set both masks; use standard CAN frame
//  CAN.init_Filt(0, 0, CAN_THROTTLE_MSG_ADDRESS);              //filter 0 for receive buffer 0
//  CAN.init_Filt(2, 0, CAN_THROTTLE_MSG_ADDRESS);              //filter 1 for receive buffer 1               
//
//  
//  delay(250);   //delay after setting filters and masks before getting the zero position of the hall sensor.
//
//  findHallZeroPosition();

}

void loop()
{
  
//  calculateCurrent();
//  calculateVoltage();
//  calculateTemperature();
//  getZeroedHallPosition();
//  sendCanMsg();
//  receiveCanMsg();
//
//  Serial.print("Input: ");
//  Serial.print(throttle_input);
//  Serial.print("\tFeedback: ");
//  Serial.print(zeroedHallPosition_percentx10/10.0);
//  Serial.print("\tCurrent (mA): ");
//  Serial.print(currentDraw_mA);
//  Serial.print("\tVoltage: ");
//  Serial.print(voltage_mV);
//  Serial.print("\tTemperature: ");
//  Serial.println(temp_degCx2/2.0);
  
  // Check for timeout then execute controls loop
  if(millis() - last_recv > 500){
    
    // timeout the controls
    throttle_input = 50;
    
  }
  Serial.print( F(" | Clean angle: ") );
  Serial.print( Sensor.getAngle() );
  if ( Serial.read() == '0' )
    {
        Sensor.setZeroPosition();
        Serial.print( F(" | Zero position set!") );
    }
  // set the inb low
  digitalWrite(4, 1);

  // set the ina high
  digitalWrite(5, 0);

  // set the PWM to half
  Throt = analogRead(A0);
  Serial.println(Throt/1024.0*255.0);
  analogWrite(6, Throt/1024.0*255.0);

  delay(500);
  
}
