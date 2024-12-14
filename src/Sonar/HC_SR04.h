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
    static int count;
    int id;
    //static std::map<int, long long> startTime; // echo : startTime
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
     */
    HC_SR04(int trigger, int echo);
    ~HC_SR04();

    /**
     * @par timeout useless
     * @return distance in centimetres from sensor, if sensor times out it returns 0 cm
     */
    double Distance(int timeout);
    
    /**
     * @par timeout_ms in miliseconds
     * @return sensor timeout in us if the sensor is positioned in direction when nothing is within the range of sensor
     */
    long long MeassureSensorTimeout(int timeout_ms);
};



#endif //HC_SR04_H
