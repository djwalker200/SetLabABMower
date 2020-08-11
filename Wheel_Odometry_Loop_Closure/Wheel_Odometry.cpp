//simple class file used to simplify a given orientation of the robot
#include "Wheel_Odometry.h"
#include <vector>
#include <math.h>   
#include <iostream>



/* Function that returns the radius of curvature given the current velocities of each wheel
a positive return value represents a center of curvature to the left of the robot*/
double Calculate_Turn_Radius(){
    //in the case that both sides are equal, creates a non-infinite return value to signify a straight line movement
    if(Left_Velocity == Right_Velocity) return 10000;
    
    return (axle_length / 2) * (Right_Velocity + Left_Velocity) / (Right_Velocity - Left_Velocity);

}

// Function that returns the rate at which the robot will rotate about the center of curvature
double Calculate_Rotation_Velocity(){
    return (Right_Velocity - Left_Velocity) / axle_length;
    
}

bool Check_Power(){
    /*implement power check from robot battery
    ROBOT_ONLINE = digitalRead(PIN);
    */

   return ROBOT_ONLINE;

}
/*Function that returns the chane in orientation of the robot given the most recent orientation 
of the robot and the velocities of each wheel */
Robot_Pose Calculate_Delta(Robot_Pose last){
    //The distance to the center of curvature
    double radius = Calculate_Turn_Radius();
    //The angular velocity about the center of curvature
    double rotation_velocity = Calculate_Rotation_Velocity();
    //The current angle to the x axis of the starting reference frame
    double theta = last.get_Angle();
    //Angle that would be rotated about the center of curvature during the time interval
    double rotation_angle = rotation_velocity * TIME_INTERVAL;

    //
    if(radius == 10000){
        double delta_X = Left_Velocity * TIME_INTERVAL * cos(theta);
        double delta_Y = Left_Velocity * TIME_INTERVAL * sin(theta);
        return Robot_Pose(delta_X, delta_Y, 0);

    }

    //The displacement in the x direction with respect to the starting reference frame
    double delta_X = radius * ( sin(theta) * cos(rotation_angle) + cos(theta) * sin(rotation_angle) - sin(theta) );
    //The displacement in the y direction with respect to the starting reference frame
    double delta_Y = radius * ( sin(theta) * sin(rotation_angle) + cos(theta) * cos(rotation_angle) + cos(theta) );
    
    //Combines the estimated dispacements with the most recent location measurements to update the location
    return Robot_Pose(delta_X, delta_Y, rotation_angle);



}


int main(){
    Robot_Pose current, delta;
    std::vector<Robot_Pose> robot_Path;
    robot_Path.push_back(current);
    std::vector<Robot_Pose> deltas;
    while(ROBOT_ONLINE){
        /*
        Read in Left and Right Wheel Velocities
        Left_Velocity = digitalRead(LEFT_PIN);
        Right_Velocity = digitalRead(RIGHT_PIN);
        */
       //if no slipping detected, update position estimate using wheel odometry
       if(!WHEEL_SLIP_DETECTED){
           delta = Calculate_Delta(current);
           deltas.push_back(delta);
           current += delta;
           robot_Path.push_back(current);
       }
       else{
           /*implement alternative method for when wheels are slipping
           current method assumes the robot did not move or rotate during while slipping*/
       }

    }

    //made up test cases to confirm algorithm

    //vector to store the different calculated orientations of the robot over time
    std::vector<Robot_Pose> Robot_Positions;
    Robot_Positions.push_back(Robot_Pose());
    //vector to store all of the changes between orientations of the robot over time
    std::vector<Robot_Pose> path_Deltas;
    //made up test numbers for the values outputted by the rotary encoders
    std::vector<double> right_velos, left_velos;
    right_velos.push_back(10);
    right_velos.push_back(2);
    right_velos.push_back(10);
    right_velos.push_back(2);
    right_velos.push_back(10);
    right_velos.push_back(2);
    right_velos.push_back(10);
    right_velos.push_back(2);

    left_velos.push_back(10);
    left_velos.push_back(-2);
    left_velos.push_back(10);
    left_velos.push_back(-2);
    left_velos.push_back(10);
    left_velos.push_back(-2);
    left_velos.push_back(10);
    left_velos.push_back(-2);

    for(int i = 0; i < right_velos.size(); i++){
        Right_Velocity = right_velos[i];
        Left_Velocity = left_velos[i];
        current = Robot_Positions[i];
        delta = Calculate_Delta(current);
        path_Deltas.push_back(delta);
        current += delta;
        Robot_Positions.push_back(current);
    }

    for(int i = 0; i < Robot_Positions.size(); i++){
        std::cout << Robot_Positions[i] << std::endl;
    }
    std::cout << std::endl << std::endl;

    std::vector<Robot_Pose> corrected = Weighted_Loop_Closure(Robot_Positions,path_Deltas);

    for(int i = 0; i < corrected.size(); i++){
        std::cout << corrected[i] << std::endl;
    }
    


    return 0;
}