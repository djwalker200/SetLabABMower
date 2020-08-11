

void Update_Velocities(){
  Right_Velocity = analogRead(ENAPin);
  Left_Velocity = analogRead(ENBPin);
  //Create a Yaw sensor pin in MEGA.ino
  //Yaw = analogRead(YAWPin);
}
//Sends the current Velocities and Yaw measurements to Raspberry PI
void Send_Velocities(){
  Serial.write(Right_Velocity);
  Serial.write(Left_Velocity);
  Serial.write(Yaw);
}
//Reads an updated wheel odometry measurement from the Raspberry PI
void Update_Odometry(){
  current_X = Serial.read();
  current_Y = Serial.read();
  current_Angle = Serial.read();
}
    
