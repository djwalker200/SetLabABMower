#ifndef Wheel_Odometry_h
#define Wheel_Odometry_h
// helper class used to store the current pose of the robot, < x_coordinate, y_coordinate, angle >
#include <math.h>
#include <iostream>
#include <vector>

//constant for 2PI
double TWO_PI = 2 * M_PI;

//true if robot power is currently on and all communications are successful
bool ROBOT_ONLINE = false;
//true if wheel slippage is detected, false if no slippage is detected
bool WHEEL_SLIP_DETECTED = false;
/*time interval in seconds that a new position will be calculated
 this should be tuned to the return frequency of the wheel encoders*/
const double TIME_INTERVAL = 0.1;

// horizontal distance between the two wheels, measured in meters
const double axle_length = 0.25;

//angular velocities of each wheel, meased in radians per second, to be read in from the I2C bus
double Right_Velocity = 0.0;
double Left_Velocity = 0.0;
//pins to read values from the motor encoders
//pin for checking if robot is on
int PIN = 1;
int LEFT_PIN = 2;
int RIGHT_PIN = 3;



//Class to represent a single orientation of the robot (x,y,angle) relative to origin reference frame
class Robot_Pose{
public:
    //default constructor to set robot to origin (charge station)
    Robot_Pose() : x_coordinate(0.0), y_coordinate(0.0), angle(0.0) {}
    //constructor
    Robot_Pose(double x, double y, double theta) : x_coordinate(x), y_coordinate(y), angle(theta) {}
    //modifier functions
    void set_X(double x) { x_coordinate = x;}
    void set_Y(double y) { y_coordinate = y;}
    void set_Angle(double theta) {angle = theta;}
    //accessor functions
    double get_X() const { return x_coordinate;}
    double get_Y() const { return y_coordinate;}
    double get_Angle() const { return angle;}
    //subtraction operator overload 
    Robot_Pose operator-(const Robot_Pose& r2);
    //decrement operator overload
    Robot_Pose& operator-=( const Robot_Pose& r2);
    //addition operator overload 
    Robot_Pose operator+(const Robot_Pose& r2);
    //increment operator overload
    Robot_Pose& operator+=( const Robot_Pose& r2);
    //coefficient multiplication operator overload
    Robot_Pose operator*(float x);
    //coefficient division operator overload
    Robot_Pose operator/(float x);

private:
    double x_coordinate;
    double y_coordinate;
    double angle;
};
std::ostream& operator<<(std::ostream& os, Robot_Pose& r1);


//subtraction operator overload
Robot_Pose Robot_Pose::operator-(const Robot_Pose& r2){
    return Robot_Pose(this->get_X() - r2.get_X(), this->get_Y() - r2.get_Y(), fmod(this->get_Angle() - r2.get_Angle(),TWO_PI));
}
//decrememt operator overload
Robot_Pose& Robot_Pose::operator-=(const Robot_Pose& r2){
    this->set_X(this->get_X() - r2.get_X());
    this->set_Y(this->get_Y() - r2.get_Y());
    this->set_Angle(fmod(this->get_Angle() - r2.get_Angle(),TWO_PI));
    return *this; 
}
//addition operator overload
Robot_Pose Robot_Pose::operator+(const Robot_Pose& r2){
    return Robot_Pose(this->get_X() + r2.get_X(), this->get_Y() + r2.get_Y(), fmod(this->get_Angle() + r2.get_Angle(),TWO_PI));
}
//increment operator overload
Robot_Pose& Robot_Pose::operator+=(const Robot_Pose& r2){
    this->set_X(this->get_X() + r2.get_X());
    this->set_Y(this->get_Y() + r2.get_Y());
    this->set_Angle(fmod(this->get_Angle() + r2.get_Angle(),TWO_PI));
    return *this;
}

//scales the values of a pose by up by a constant
Robot_Pose Robot_Pose::operator*(float c){
    return Robot_Pose(this->get_X() * c, this->get_Y() * c, fmod(this->get_Angle() * c,TWO_PI));
}
//scales the values of a pose down by a constant
Robot_Pose Robot_Pose::operator/(float c){
    return Robot_Pose(this->get_X() / c, this->get_Y() / c, fmod(this->get_Angle() / c,TWO_PI));
}
//used for printing 
std::ostream& operator<<(std::ostream& os, Robot_Pose& r1){
    os << "< " << r1.get_X() << " , " << r1.get_Y() << " , " << (int)(r1.get_Angle() * 180 / M_PI) << " >";
    return os;
}
//uses the standard loop closure method to make corrections to the loop traveled
std::vector<Robot_Pose> const Loop_Closure(std::vector<Robot_Pose> path, std::vector<Robot_Pose> deltas){
    //number of calulated points along the loop
    int num_points = path.size();
    //the total error created along the loop
    Robot_Pose error = path[num_points - 1] - path[0];
    //A scaled error, evenly distributed across each point of the loop
    Robot_Pose scaled_Error = error / num_points;
    //vector to store the new corrected delta values
    std::vector<Robot_Pose> corrected_Deltas = deltas;
    //corrects each of the deltas throughout the loop evenly with respect to the total error
    for(int i = 0; i < deltas.size(); i++){
        corrected_Deltas[i] -= scaled_Error;
    }
    //vector to store the new corrected path
    std::vector<Robot_Pose> updated_Path;
    updated_Path.push_back(path[0]);
    //recreates the path corrected for the error in the loop
    for(int i = 0; i < corrected_Deltas.size(); i++){
        updated_Path.push_back(updated_Path[i] + corrected_Deltas[i]);
    }
    return updated_Path;

}

//alternative loop closure call for if deltas were not kept track of 
std::vector<Robot_Pose> const Loop_Closure(std::vector<Robot_Pose> path){
    std::vector<Robot_Pose> deltas;
    for(int i = 1; i < path.size(); i++){
        deltas.push_back(path[i] - path[i - 1]);
    }
    return Loop_Closure(path,deltas);
}


//uses the weighted loop closure method to account for error within a loop
std::vector<Robot_Pose> const Weighted_Loop_Closure(std::vector<Robot_Pose> path, std::vector<Robot_Pose> deltas){
    //number of calculated points along the path
    int num_points = path.size();
    //the total error created along the loop
    Robot_Pose error = path[num_points - 1] - path[0];
    //vector to store the new corrected delta values
    std::vector<Robot_Pose> corrected_Deltas = deltas;
    std::vector<double> magnitudes;
    //tracks the sum of all magnitudes of the deltas
    double total_magnitude = 0;
    //placeholder variable to store the magnitude of each delta
    double magnitude;
    //tracks the sum of all angle deltas (absolute value)
    double total_angle_magnitude = 0;
    //loops through each delta, finding and storing the magnitude of the displacement and the angle
    for(int i = 0; i < deltas.size(); i++){
        //uses m = |p2 - p1|^ 2
        magnitude = pow(deltas[i].get_X(),2) + pow(deltas[i].get_Y(),2);
        //stores each magnitude to be used later
        magnitudes.push_back(magnitude);
        //adds each magnitude to the running sum
        total_magnitude += magnitude;
        //adds the magnitude of the change in angle to the running sum
        total_angle_magnitude += abs(deltas[i].get_Angle());
    }
    //weight value for each movement along the path
    double weight;
    for(int i = 0; i < deltas.size(); i++){
        //sets the corrected weighted position deltas
        weight = magnitudes[i] / total_magnitude;
        corrected_Deltas[i] -= error * weight;
        //sets the corrected weighted angle deltas
        weight = abs(deltas[i].get_Angle()) / total_angle_magnitude;
        corrected_Deltas[i].set_Angle(deltas[i].get_Angle() - error.get_Angle() * weight);
    }
    //vector to store the corrected path
    std::vector<Robot_Pose> updated_path;
    //adds the starting point to the updated path
    updated_path.push_back(path[0]);
    //recreates the path accounting for the error in the loop
    for(int i = 0; i < corrected_Deltas.size(); i++){
        updated_path.push_back(updated_path[i] + corrected_Deltas[i]);
    }
    return updated_path;

}

//alternative weighted loop closure method for when deltas are not kept track of

std::vector<Robot_Pose> const Weighted_Loop_Closure(std::vector<Robot_Pose> path){
    std::vector<Robot_Pose> deltas;
    for(int i = 1; i < path.size(); i++){
        deltas.push_back(path[i] - path[i - 1]);
    }
    return Weighted_Loop_Closure(path, deltas);
}
#endif