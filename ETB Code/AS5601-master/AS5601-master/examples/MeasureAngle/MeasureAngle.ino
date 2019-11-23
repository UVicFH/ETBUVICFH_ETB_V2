/**
 * Measure Angle
 * with AS5601 Magnet Rotary Sensor
 */


#include <Arduino.h>
#include "AS5601.h"


AS5601 Sensor;

// NCP18XH103F03RB
int TempSens = A2;
float Temperature = 0;
float Temperature_v = 0;
float Temperature_r = 0;

int VoltSens = A0;
float Voltage = 0;

int CurrSens = A1;
float Current = 0;


int motor_ina = 4;
int motor_inb = 5;
int motor_pwm = 6;

void setup()
{
    delay( 1000 );
    Serial.begin( 115200 );
    Serial.println( F("Angle Measurement with AS5601") );

    //Motor Controller
    pinMode(motor_ina, OUTPUT);
    pinMode(motor_inb, OUTPUT);
    pinMode(motor_pwm, OUTPUT);
    
}


void loop()
{

    //Hall Sensor
    Serial.print( F("Magnet?: ") );
    Serial.print( Sensor.magnetDetected() );
    
    Serial.print( F("Magnitude: ") );
    Serial.print( Sensor.getMagnitude() );

    Serial.print( F(" | Raw angle: ") );
    Serial.print( Sensor.getRawAngle() );

    Serial.print( F(" | Clean angle: ") );
    Serial.print( Sensor.getAngle() );
    Serial.println();
    
    if ( Serial.read() == '0' )
    {
        Sensor.setZeroPosition();
        Serial.print( F(" | Zero position set!") );
    }

    //Temperature Sensor
    Temperature_v = (((float)analogRead(TempSens))/1024.0)*5.0;
    Temperature_r = (5000*5)/Temperature_v-5000;
    Temperature = (1.0/((1.0/(273.15+25.0))+(1/3420.0)*(log(Temperature_r/10000.0))))-273.15;
    Serial.print("Temperature: ");
    Serial.println(Temperature);

    //Current Sensor
    Current = ((float(analogRead(CurrSens))/1024.0)*5.0)/(3300.0*0.05*.01);
    Serial.print("Current: ");
    Serial.println(Current);

    //Voltage Sensor
    Voltage = ((float(analogRead(VoltSens))/1024.0)*5.0)*(13.3/3.3);
    Serial.print("Voltage: ");
    Serial.println(Voltage);
    
    //Motor Controller
    digitalWrite(motor_ina, 1);
    digitalWrite(motor_inb, 0);
    analogWrite(motor_pwm, 20);

    Serial.println();
    delay( 500 );
}
