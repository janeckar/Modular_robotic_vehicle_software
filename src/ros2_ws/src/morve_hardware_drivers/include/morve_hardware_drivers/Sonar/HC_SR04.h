/*************************
* name: Radek Janečka    *
* email: xjanec34@vut.cz *
* date: 17.11.2024       *
**************************/

// Inspired from: http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp

#ifndef HC_SR04_H
#define HC_SR04_H

#include <map>

/**
 * @brief In the moment only one instance of this class can exist
 */
class HC_SR04 {

  private:
    static volatile long long startTimeUsec;
    static volatile long long travelTimeUsec;

    int trigger;
    static int echo;
    double distance_cm;
    
    void SendTrigger();
    static void EchoInterrupt();

  public:
    /**
     * @brief create object of HC_SR04 sonar
     * @par trigger GPIO pin number
     * @par echo GPIO pin number
     * 
     * @throw hardware_error if the GPIO pin ECHO has already registered system interrupt
     */
    HC_SR04(int trigger, int echo);
    ~HC_SR04();

    /**
     * @par measurement_time_ms in miliseconds minimal measurement time is always 30 ms
     * @return distance in centimetres from sensor, if sensor times out it returns 0 cm
     */
    double Distance(int measurement_time_ms);
    
    /**
     * @par measurement_time_ms in miliseconds minimal measurement time is always 30 ms
     * @return sensor timeout in us if the sensor is positioned in direction when nothing is within the range of sensor
     */
    long long MeassureSensorTimeout(int measurement_time_ms);
};



#endif //HC_SR04_H
