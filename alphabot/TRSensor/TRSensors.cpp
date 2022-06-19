/*  QTRSensors.h - Library for using Pololu QTR reflectance
    sensors and reflectance sensor arrays
 * Copyright (c) 2008-2012 waveshare Corporation. For more information, see
 *
 *   http://www.waveshare.com
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact.  
 *
 * Disclaimer: To the extent permitted by law, waveshare pr
 ovides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */


#include "mbed.h"
#include "TRSensors.h"

#define NUMSENSORS 5

SPI spi(D11, D12, D13);     // D11, D12, D13
DigitalOut spi_cs(D10,1);   // PB_6: D10

// Base class data member initialization (called by derived class init())
TRSensors::TRSensors()
{
    spi.format(16,0);       // 16-bit mode, mode 0
    //spi.frequency(2000000);
    spi.frequency(4000000); // 2 MHz

    _numSensors = NUMSENSORS;
    
    calibratedMin = (unsigned int*)malloc(sizeof(unsigned int) * _numSensors);
    calibratedMax = (unsigned int*)malloc(sizeof(unsigned int) * _numSensors);
    
    for(int i=0;i<_numSensors;i++)
    {
        calibratedMin[i] = 1023;
        calibratedMax[i] = 0;
    }
}


// Reads the sensor values using TLC1543 ADC chip into an array. 
// The values returned are a measure of the reflectance in abstract units,
// with higher values corresponding to lower reflectance (e.g. a black
// surface or a void).
void TRSensors::AnalogRead(unsigned int *sensor_values)
{
    char i,j;
    unsigned int channel = 0;
    unsigned int values[] = {0,0,0,0,0,0,0,0}; // array

    for(j = 0;j < _numSensors + 1;j++) 
    {
        spi_cs = 0;
        wait_us(2);
        values[j] = spi.write(j << 12);
        spi_cs = 1;
        wait_us(21);
    }

    for(i = 0;i < _numSensors;i++)
    {
        sensor_values[i] = values[i+1];
    }
}

// Reads the sensors 10 times and uses the results for
// calibration.  The sensor values are not returned; instead, the
// maximum and minimum values found over time are stored internally
// and used for the readCalibrated() method.
void TRSensors::calibrate()
{
    int i;
    unsigned int sensor_values[_numSensors];
    unsigned int max_sensor_values[_numSensors];
    unsigned int min_sensor_values[_numSensors];

    int j;
    for(j=0;j<10;j++)
    { 
        AnalogRead(sensor_values);
        for(i=0;i<_numSensors;i++)
        {
            // set the max we found THIS time
            // j==0일때 무조건 true니까 초기에 max-sensor-value 값이 sensor-values값으로 세팅된다.
            // 이후에 sensor_values의 값이 이전 데이터보다 클 경우에만 max-sensor-value가 sensor-values 값으로 세팅된다.
            // i가 Sensor 넘버
            // 근데 j를 0에서부터 9까지 돌리는 이유? ★★★★★★ --> 총 10번
            if(j == 0 || max_sensor_values[i] < sensor_values[i])
                max_sensor_values[i] = sensor_values[i];

            // set the min we found THIS time
            // 위와 동일한 원리로 적용
            if(j == 0 || min_sensor_values[i] > sensor_values[i])
                min_sensor_values[i] = sensor_values[i];
        }
    }
  
    //위 코드 다 돌아가면 나온다.
    //위의 코드가 각 센서마다 값이 설정되는건가보다.
    //위 코드 다 돌아가면 max_sensor_values 값과 min_sensor_values 값이 결정된다.
  // record the min and max calibration values
  for(i=0;i<_numSensors;i++)
  {
    if(min_sensor_values[i] > calibratedMax[i])
      calibratedMax[i] = min_sensor_values[i];
    if(max_sensor_values[i] < calibratedMin[i])
      calibratedMin[i] = max_sensor_values[i];
  }
}


// Returns values calibrated to a value between 0 and 1000, where
// 0 corresponds to the minimum value read by calibrate() and 1000
// corresponds to the maximum value.  Calibration values are
// stored separately for each sensor, so that differences in the
// sensors are accounted for automatically.
void TRSensors::readCalibrated(unsigned int *sensor_values)
{
    int i;

    // read the needed values
    AnalogRead(sensor_values);

    for(i=0;i<_numSensors;i++)
    {
        unsigned int calmin,calmax;
        unsigned int denominator;

        denominator = calibratedMax[i] - calibratedMin[i];

        signed int x = 0;
        if(denominator != 0)
            x = (((signed long)sensor_values[i]) - calibratedMin[i])
                * 1000 / denominator;
        if(x < 0)
            x = 0;
        else if(x > 1000)
            x = 1000;
        sensor_values[i] = x;
        // sensor_values[i]는 0-1000사이의 값으로 scaling되어 나온다.
    }

}


// Operates the same as read calibrated, but also returns an
// estimated position of the robot with respect to a line. The
// estimate is made using a weighted average of the sensor indices
// multiplied by 1000, so that a return value of 0 indicates that
// the line is directly below sensor 0, a return value of 1000
// indicates that the line is directly below sensor 1, 2000
// indicates that it's below sensor 2000, etc.  Intermediate
// values indicate that the line is between two sensors.  The
// formula is:
// 
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
//
// By default, this function assumes a dark line (high values)
// surrounded by white (low values).  If your line is light on
// black, set the optional second argument white_line to true.  In
// this case, each sensor value will be replaced by (1000-value)
// before the averaging.
int TRSensors::readLine(unsigned int *sensor_values, unsigned char white_line)
{
    unsigned char i, on_line = 0;
    unsigned long avg; // this is for the weighted total, which is long
                       // before division
    unsigned int sum; // this is for the denominator which is <= 64000
    static int last_value=0; // assume initially that the line is left.

    readCalibrated(sensor_values);

    avg = 0;
    sum = 0;
  
    for(i=0;i<_numSensors;i++) {
        int value = sensor_values[i];

        if(!white_line)
            value = 1000-value;
        sensor_values[i] = value;
        // keep track of whether we see the line at all
        if(value > 300) {
            on_line = 1;
        }
        
        // only average in values that are above a noise threshold
        if(value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
         if(last_value < (_numSensors-1)*1000/2)    // 2000보다 last value가 작으면 0 리턴 / last value 최종 위치
             return 0;
        
        // If it last read to the right of center, return the max.
         else
             return (_numSensors-1)*1000;   // 2000보다 last value가 크면 4000 리턴
    }

    last_value = avg/sum;

    return last_value;
}
