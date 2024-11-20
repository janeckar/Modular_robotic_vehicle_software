/*************************
* name: Radek Janečka    *
* email: xjanec34@vut.cz *
* date: 17.11.2024       *
**************************/

#ifndef HC_SR04_H
#define HC_SR04_H


class HC_SR04 {

  private:
    int trigger;
    int echo;
    // used from: http://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Cpp
    // slightly modified
    //volatile for the two lines below
    long long startTimeUsec;
    long long endTimeUsec;
    double distance_cm;
    long long travelTimeUsec;
    long long now;
    void recordPulseLength();
    // end

    int Travel_time_ms();

  public:
    /**
     * @brief create object of HC_SR04 sonar
     * @par trigger GPIO pin number
     * @par echo GPIO pin number
     */
    HC_SR04(int trigger, int echo);

    double Distance(int timeout);
};



#endif //HC_SR04_H
