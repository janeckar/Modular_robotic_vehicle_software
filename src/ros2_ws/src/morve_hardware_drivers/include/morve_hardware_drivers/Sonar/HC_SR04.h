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
    double distance_m;
    
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
     * @brief Blocking method waiting for distance measurement for specified time.
     * 
     * @par measurement_time_ms in miliseconds minimal measurement time is always 30 ms
     * @return distance in metres from sensor, if sensor times out it returns 0 m
     */
    double Distance(int measurement_time_ms);
    
    /**
     * @brief Blocking method waiting for result for the specified measurement time.
     * @par measurement_time_ms in miliseconds minimal measurement time is always 30 ms
     * @return sensor timeout in us if the sensor is positioned in direction when nothing is within the range of sensor
     */
    long long MeassureSensorTimeout(int measurement_time_ms);

    /**
     * @brief Non-blocking method which will invoke distance measurement.
     * 
     * The minimal time for measurement to take place and the ultrasound to be dispered
     * is 30 ms or period of 33 or less measurements per second.
     * 
     * This method only invokes the measurement so it does not block. To read the value after the measurement time
     * use the read_distance() method.
     */
    void request_distance();

    /**
     * @brief Non-blocking method which will calculates distance returned after calling request_distance.
     * 
     * Call this method minimaly 30 ms after the request_distance. After this call the request can be called imidietly again
     * but the measured distance is reset.
     * 
     * If the measurement was not ready the output distance is zero.
     * 
     * @return double distance in metres
     */
    double read_distance(); 
};



#endif //HC_SR04_H
