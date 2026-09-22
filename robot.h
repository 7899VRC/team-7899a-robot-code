#ifndef ROBOT_H
#define ROBOT_H

#include <vector>


class Robot {
  public:
    // Assign hardware interfaces for the robot
    std::vector<motor> LEFT_MOTORS = {
      motor(PORT12, ratio18_1, false)
      //motor(PORT19, ratio18_1, true),
      //motor(PORT20, ratio18_1, true),
    };

    std::vector<motor> RIGHT_MOTORS = {
      motor(PORT20, ratio18_1, true)
      //motor(PORT12, ratio18_1, false),
      //motor(PORT11, ratio18_1, false),
    };

    distance RIGHT_DISTANCE = distance(PORT3);
    distance LEFT_DISTANCE = distance(PORT9);
    inertial IMU = inertial(PORT8);

    float INTERVAL_TIME = 0.03; // Time per control interval in seconds

    float TRACK_WIDTH = 10; // Width between the center of left and right wheels
    float WHEEL_DIAMETER = 2.75; // Diameter of wheels
    float EXTERNAL_RATIO = 1; // External gear ratio between motor and wheel
    float MAX_RPM = 200; // Max RPM based on motor type
    float MIN_PERCENT = 5; // Minimum percent required to move the robot
    float CENTER_TO_DISTANCE = 5; // Distance from robot center to distance sensor

    Pose current = Pose(0, 0, 0); // Current estimated robot pose (absolute)
    Pose offset = Pose(0, 0, 0); // Most recent commanded offset (relative)
    float percent = 30; // Maximum target motor percent
    bool rampUp = true; // Option to gradually increase to maximum speed
    bool rampDown = true; // Option to gradually decrease from maximum speed

    Robot() {
      // Reset all hardware interfaces when robot is initialized
      for (auto leftMotor : this->LEFT_MOTORS) {
        leftMotor.resetPosition();
      }

      for (auto rightMotor : this->RIGHT_MOTORS) {
        rightMotor.resetPosition();
      }

      // Run IMU calibration until it is finished
      // Make sure to not touch the robot when the program starts
      this->IMU.calibrate(); 
      while (IMU.isCalibrating()) {
        wait(this->INTERVAL_TIME, sec);
      }
    }

    void leftDrive(float percent) {
      if (percent < 0) {
        percent = fmax(-100, percent); // Lowest negative speed is -100
        percent = fmin(-this->MIN_PERCENT, percent); // Highest negative speed is -MIN_PERCENT
      }
      else {
        percent = fmax(this->MIN_PERCENT, percent); // Lowest positive speed is MIN_PERCENT
        percent = fmin(100, percent); // Highest positive speed is 100
      }

      // Spin all left motors at the same speed
      for (auto leftMotor : this->LEFT_MOTORS) {
        leftMotor.spin(forward, percent, pct);
      }
    }

    void rightDrive(float percent) {
      if (percent < 0) {
        percent = fmax(-100, percent); // Lowest negative speed is -100
        percent = fmin(-this->MIN_PERCENT, percent); // Highest negative speed is -MIN_PERCENT
      }
      else {
        percent = fmax(this->MIN_PERCENT, percent); // Lowest positive speed is MIN_PERCENT
        percent = fmin(100, percent); // Highest positive speed is 100
      }

      // Spin all right motors at the same speed
      for (auto rightMotor : this->RIGHT_MOTORS) {
        rightMotor.spin(forward, percent, pct);
      }
    }

    void leftBrake() {
      // Stop all left motors
      for (auto leftMotor : this->LEFT_MOTORS) {
        leftMotor.stop(brake);
      }
    }

    void rightBrake() {
      // Stop all right motors
      for (auto rightMotor : this->RIGHT_MOTORS) {
        rightMotor.stop(brake);
      }
    }

    void stopAll() {
      // Stop all motors
      leftBrake();
      rightBrake();
    }

    float getLeftRotation() {
      // Read all left motor encoders
      // Use the average if there are multiple motors
      float total = 0;
      for (auto leftMotor : this->LEFT_MOTORS) {
        total += leftMotor.position(degrees);
      }

      return total / this->LEFT_MOTORS.size();
    }

    float getRightRotation() {
      // Read all right motor encoders
      // Use the average if there are multiple motors
      float total = 0;
      for (auto rightMotor : this->RIGHT_MOTORS) {
        total += rightMotor.position(degrees);
      }

      return total / this->RIGHT_MOTORS.size();
    }

    float getLeftDistance() {
      // Read the left distance sensor
      // Include the distance from the sensor to the center of the robot
      return this->LEFT_DISTANCE.objectDistance(inches) + this->CENTER_TO_DISTANCE;
    }

    float getRightDistance() {
      // Read the right distance sensor
      // Include the distance from the sensor to the center of the robot
      return this->RIGHT_DISTANCE.objectDistance(inches) + this->CENTER_TO_DISTANCE;
    }

    void setCurrent(Pose current) {
      // Set the current estimated robot pose (absolute)
      this->current = current;
    }

    void setOffset(Pose offset) {
      // Set the most recent commanded offset (relative)
      this->offset = offset;
    }
    
    void setPercent(float percent) {
      // Configure maximum target motor percent
      this->percent = percent;
    }
    
    void setRampUp(bool rampUp) {
      // Configure option to gradually increase to maximum speed
      this->rampUp = rampUp;
    }
    
    void setRampDown(bool rampDown) {
      // Configure option to gradually decrease from maximum speed
      this->rampDown = rampDown;
    }

    float percentToSpeed(float percent) {
      // Convert motor percent to linear speed
      return (percent * this->MAX_RPM * M_PI * this->WHEEL_DIAMETER * this->EXTERNAL_RATIO) / (60 * 100);
    }

    float speedToPercent(float speed) {
      // Convert linear speed to motor percent
      return (speed * 60 * 100) / (this->MAX_RPM * M_PI * this->WHEEL_DIAMETER * this->EXTERNAL_RATIO);
    }

    float getMaxSpeed() {
      // Get maximum speed based on maximum target motor percent
      return percentToSpeed(this->percent);
    }
    
    // Given progress, which is a number from 0 to 1 representing percent of motion completed
    // return interval control time. Time may be stretched if ramp up or ramp down is enabled.
    float getTime(float progress) {
      float maxTime = 3 * this->INTERVAL_TIME; // Max time is 3 times normal interval time
      float t = 0;
  
      if (progress < 0.1) { // When progress is less than 10%
        if (this->rampUp) {
          // If ramp up is enabled, gradually decrease from max time to normal interval time
          t = -(maxTime / 0.1) * progress + maxTime + this->INTERVAL_TIME;
        }
        else {
          // If ramp up is not enabled, use normal interval time
          t = this->INTERVAL_TIME;
        }
      }
      else if (progress < 0.9) { // When progress is less than 90%
        // Use normal interval time
        t = this->INTERVAL_TIME;
      }
      else { // When progress is 90% or more
        if (this->rampDown) {
          // If ramp down is enabled, gradually increase from normal interval time to max time
          t = (maxTime / 0.1) * (progress - 1) + maxTime + this->INTERVAL_TIME;
        }
        else {
          // If ramp down is not enabled, use normal interval time
          t = this->INTERVAL_TIME;
        }
      }
  
      return t;      
    }
    
    // Calculate left and right velocities required to reach a pose
    Command getPoseCommand(Pose offset, float t) {
      setOffset(offset);

      float v = 0;
      float w = 0;
      
      if (offset.y != 0) {
        // If there is any lateral offset, calculate curve required to reach x, y
        float r = ((offset.x * offset.x) + (offset.y * offset.y)) / (2 * offset.y);
        float th = sqrt((offset.x * offset.x) + (offset.y * offset.y)) / r;

        // Calculate linear and angular velocity
        v = (r * th) / t;
        w = th / t;

        // Reverse linear and angular velocity if our target is backwards
        if (offset.x < 0) {
          v = -v;
          w = -w;
        }
      }
      else if (offset.x != 0) {
        // Only linear offset, drive straight
        // We only have linear velocity
        v = offset.x / t;
        w = 0; 
      }
      else {
        // No lateral or linear offset, turn in place
        // We only have angular velocity
        v = 0;
        w = offset.rad / t;
      }

      return Command(v, w);
    }

    void execute(Command command) {
      if (command.v || command.w) {
        // Calculate left and right wheel velocities based on linear and angular velocity
        float vl = command.v - ((command.w * this->TRACK_WIDTH) / 2);
        float vr = command.v + ((command.w * this->TRACK_WIDTH) / 2);

        leftDrive(speedToPercent(vl));
        rightDrive(speedToPercent(vr));
      }
    }

    // Drive to any x, y position in time t
    // Angle is not included unless we are turning in place
    void driveToPose(Pose offset, float t) {
      Command command = getPoseCommand(offset, t);
      execute(command);
    }

    // Drive in a straight line at max motor percent
    void straight(float targetDistance) {
      float totalDistance = 0;
      float newDistance = 0;
      float t = 0;
      
      float speed = getMaxSpeed();
      // If our target is backwards, calculate with negative speed
      if (targetDistance < 0) {
        speed = -speed;
      }

      // Control until cumulative distance reaches target distance
      while (fabs(totalDistance) <= fabs(targetDistance)) {
        // Calculate a small distance for one control interval
        newDistance = speed * this->INTERVAL_TIME;
        // Time is normal control time
        t = this->INTERVAL_TIME;
        // Send drive command for small distance
        driveToPose(Pose(newDistance, 0, 0), t);
        // Add small distance to cumulative distance
        totalDistance += newDistance;
        
        wait(t, sec);
      }
    }

    // Drive in a straight line with optional speed ramping
    void straightRamp(float targetDistance) {
      float totalDistance = 0;
      float newDistance = 0;
      float t = 0;
      
      float speed = getMaxSpeed();
      // If our target is backwards, calculate with negative speed
      if (targetDistance < 0) {
        speed = -speed;
      }

      // Control until cumulative distance reaches target distance
      while (fabs(totalDistance) <= fabs(targetDistance)) {
        // Calculate a small distance for one control interval
        newDistance = speed * this->INTERVAL_TIME;
        // Calculate time based on percent of distance covered
        t = getTime(totalDistance / targetDistance);
        // Send drive command for small distance
        driveToPose(Pose(newDistance, 0, 0), t);
        // Add small distance to cumulative distance
        totalDistance += newDistance;
        
        wait(t, sec);
      }
    }
    
    // Turn in place at max motor percent
    void turn(float targetAngle) {
      float totalAngle = 0;
      float newAngle = 0;
      float t = 0;
      
      float speed = getMaxSpeed();
      // If our target is clockwise, calculate with negative speed
      if (targetAngle < 0) {
        speed = -speed;
      }

      // Control until cumulative angle reaches target angle
      while (fabs(totalAngle) <= fabs(targetAngle)) {
        // Calculate a small angle for one control interval
        newAngle = toDegrees((2 * speed * this->INTERVAL_TIME) / this->TRACK_WIDTH);
        // Time is normal control time
        t = this->INTERVAL_TIME;
        // Send drive command for small distance
        driveToPose(Pose(0, 0, newAngle), t);
        // Add small angle to cumulative angle
        totalAngle += newAngle;

        wait(t, sec);
      }
    }

    // Turn in place with optional speed ramping
    void turnRamp(float targetAngle) {
      float totalAngle = 0;
      float newAngle = 0;
      float t = 0;
      
      float speed = getMaxSpeed();
      // If our target is clockwise, calculate with negative speed
      if (targetAngle < 0) {
        speed = -speed;
      }

      // Control until cumulative angle reaches target angle
      while (fabs(totalAngle) <= fabs(targetAngle)) {
        // Calculate a small angle for one control interval
        newAngle = toDegrees((2 * speed * this->INTERVAL_TIME) / this->TRACK_WIDTH);
        // Calculate time based on percent of angle covered
        t = getTime(totalAngle / targetAngle);
        // Send drive command for small angle
        driveToPose(Pose(0, 0, newAngle), t);
        // Add small angle to cumulative angle
        totalAngle += newAngle;

        wait(t, sec);
      }
    }
};


#endif