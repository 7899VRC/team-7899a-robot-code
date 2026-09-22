#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include <memory>
#include <queue>


// This is a virtual base class for Estimators
// Essentially a template that we should follow in order to implement other Estimators
class Estimator {
  public:
    // Store a pointer to a robot so we can read data from the robot
    std::shared_ptr<Robot> robot;

    // Save the most recent pose estimate within the estimator
    Pose current = Pose(0, 0, 0);
    
    // We need to include this so all of our Estimators are destroyed properly
    virtual ~Estimator() = default;

    // This is the main function that we need to implement for each Estimator
    // We will try to estimate the robot's current pose, then store that pose in the robot
    virtual void update() = 0;
};


// The simplest type of estimation
// Just assume that the command we send is the command we actually reach
class DirectEstimator : public Estimator {
  public:
    DirectEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
    }

    // Helper function to estimate a new pose based on the latest commanded offset
    Pose estimateDirect() {
      // Current pose + offset gives us a new current pose
      Pose current = absoluteTransform(this->current, this->robot->offset);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateDirect());
    }
};


// Use the IMU to estimate our angle only
class IMUEstimator : public Estimator {
  public:
    IMUEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Reset the IMU for our next reading
      this->robot->IMU.resetRotation();
    }

    // Helper function to estimate a new pose based on the latest IMU reading
    Pose estimateIMU() {
      // Normalize and multiply by -1 because the angle provided by Vex IMU is opposite our convention
      float newAngle = normalizeAngle(-1 * this->robot->IMU.rotation(degrees));
      // Reset the IMU for our next reading
      this->robot->IMU.resetRotation();

      // Apply the new angle to the current pose to create a new current pose
      Pose offset = Pose(0, 0, newAngle);
      Pose current = absoluteTransform(this->current, offset);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateIMU());
    }
};


// Use the encoders to estimate a new pose when we are traveling in a straight line
class EncoderEstimator : public Estimator {
  // Calculate expected distance vs actual distance
  float DISTANCE_FACTOR = 30.0 / 26.0;

  // Store the most recent encoder readings
  float leftRotation = 0;
  float rightRotation = 0;

  public:
    EncoderEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Take an initial reading for the encoders
      this->leftRotation = this->robot->getLeftRotation();
      this->rightRotation = this->robot->getRightRotation();
    }

    // Helper function to calculate left linear distance based on left encoders
    float calculateLeftDistance() {
      // Take a new reading from the left encoders
      float newLeftRotation = this->robot->getLeftRotation();
      // Measure the change to the new reading from the previous reading
      float leftChange = newLeftRotation - this->leftRotation;
      // Store the new reading for the next calculation
      this->leftRotation = newLeftRotation;

      // Calculate left linear distance based on the change in rotation
      float leftDistance = (
        leftChange *
        M_PI *
        this->robot->WHEEL_DIAMETER *
        this->robot->EXTERNAL_RATIO *
        this->DISTANCE_FACTOR
        ) / 360.0;

      return leftDistance;
    }

    // Helper function to calculate right linear distance based on right encoders
    float calculateRightDistance() {
      // Take a new reading from the right encoders
      float newRightRotation = this->robot->getRightRotation();
      // Measure the change to the new reading from the previous reading
      float rightChange = newRightRotation - this->rightRotation;
      // Store the new reading for the next calculation
      this->rightRotation = newRightRotation;

      // Calculate right linear distance based on the change in rotation
      float rightDistance = (
        rightChange *
        M_PI *
        this->robot->WHEEL_DIAMETER *
        this->robot->EXTERNAL_RATIO *
        this->DISTANCE_FACTOR
        ) / 360.0;

      return rightDistance;
    }

    // Helper function to estimate a new pose based on left and right distance
    Pose estimateEncoder() {
      float leftDistance = calculateLeftDistance();
      float rightDistance = calculateRightDistance();
      // Use the average of left and right because we assume we are traveling in a straight line
      float averageDistance = (rightDistance + leftDistance) / 2.0;

      // Apply only linear distance to the current pose to create a new current pose
      Pose offset = Pose(averageDistance, 0, 0);
      Pose current = absoluteTransform(this->current, offset);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateEncoder());
    }
};


// Full pose estimation using encoders
class OdometryEstimator : public Estimator {
  public:
    // Store a pointer to an EncoderEstimator so we can use encoder helper functions
    std::shared_ptr<EncoderEstimator> encoderEstimator;

    OdometryEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Save a pointer to a new EncoderEstimator
      this->encoderEstimator = std::make_shared<EncoderEstimator>(robot);
    }

    // Helper function to estimate a new pose based on left and right distance
    Pose estimateOdometry() {
      // Use the EncoderEstimator helper functions to get left and right linear distance
      float leftDistance = this->encoderEstimator->calculateLeftDistance();
      float rightDistance = this->encoderEstimator->calculateRightDistance();

      // Calculate a new pose based on the old pose and left and right linear distance
      float averageDistance = (rightDistance + leftDistance) / 2.0;
      float angleChange = (rightDistance - leftDistance) / this->robot->TRACK_WIDTH;

      Pose old = this->current;
      float x = old.x + averageDistance * cos(old.rad + (angleChange / 2.0));
      float y = old.y + averageDistance * sin(old.rad + (angleChange / 2.0));
      float th = old.deg + toDegrees(angleChange);

      // Note that these formulas give us an absolute pose, so no need to transform
      Pose current = Pose(x, y, th);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateOdometry());
    }
};


// Combine odometry estimation with IMU estimation
class OdometryIMUEstimator : public Estimator {
  public:
    // Percent to use for each odometry and IMU angle estimation, should add up to 1.0
    float PERCENT_ODOMETRY = 0.1;
    float PERCENT_IMU = 0.9;

    // Store a pointer to an OdometryEstimator so we can use odometry helper functions
    std::shared_ptr<OdometryEstimator> odometryEstimator;
    // Store a pointer to an IMUEstimator so we can use IMU helper functions
    std::shared_ptr<IMUEstimator> imuEstimator;

    OdometryIMUEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Save a pointer to a new OdometryEstimator
      this->odometryEstimator = std::make_shared<OdometryEstimator>(robot);
      // Save a pointer to a new IMUEstimator
      this->imuEstimator = std::make_shared<IMUEstimator>(robot);
    }

    // Change the ratio of percent to use for odometry and IMU angle estimation
    void setRatio(float percentOdometry, float percentIMU) {
      this->PERCENT_ODOMETRY = percentOdometry;
      this->PERCENT_IMU = percentIMU;
    }

    // Helper function to estimate a new pose based on odometry estimate and IMU estimate
    Pose estimateOdometryIMU() {
      // Get the estimated pose from the OdometryEstimator
      Pose odometryPose = this->odometryEstimator->estimateOdometry();
      // Get the estimated pose from the IMUEstimator
      Pose imuPose = this->imuEstimator->estimateIMU();

      // For x and y, just use the values provided by the OdometryEstimator
      float x = odometryPose.x;
      float y = odometryPose.y;
      // For th, use a certain percent from odometry and a certain percent from IMU
      // This is a simple form of sensor fusion
      float th = (odometryPose.deg * this->PERCENT_ODOMETRY) + (imuPose.deg * this->PERCENT_IMU);

      Pose current = Pose(x, y, th);
      this->current = current;
      this->odometryEstimator->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateOdometryIMU());
    }
};


// Try to implement some basic estimation using distance sensors
// Assume we are at exactly 90 degrees in the world
class BasicDistanceEstimator : public Estimator {
  public:
    BasicDistanceEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
    }

    // Helper function to estimate a new pose based on left and right distance sensors
    Pose estimateBasicDistance() {
      // Start with the current pose as our baseline
      Pose old = this->robot->current;
      float x = old.x;
      float y = old.y;
      float th = old.deg;

      // Read data from left and right distance sensors
      float leftDistance = this->robot->getLeftDistance();
      float rightDistance = this->robot->getRightDistance();

      // Only update our pose if we are facing straight up
      // Less than -24 means we are on the left side, use the left distance sensor
      if (old.x <= -24 && old.deg == 90 && leftDistance <= 48) {
        // The left edge reduced by our sensor reading gives us absolute x position
        x = -72 + leftDistance;
      }
      // Greater than 24 means we are on the right side, use the right distance sensor
      else if (24 <= old.x && old.deg == 90 && rightDistance <= 48) {
        // The right edge reduced by our sensor reading gives us absolute x position
        x = 72 - rightDistance;
      }

      Pose current = Pose(x, y, th);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateBasicDistance());
    }
};


// Use distance sensors to estimate x and y when we are in one of four regions
class DistanceEstimator : public Estimator {
  public:
    DistanceEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
    }

    // Helper function to estimate a new pose based on left and right distance sensors
    Pose estimateDistance() {
      // Start with the current pose as our baseline
      Pose old = this->current;
      float x = old.x;
      float y = old.y;
      float th = old.deg;

      // Read data from left and right distance sensors
      float leftDistance = this->robot->getLeftDistance();
      float rightDistance = this->robot->getRightDistance();

      // Check x and y to see if we are in any of the four regions where we can see the walls
      bool leftRegion = (-72 <= old.x && old.x <= -24 && -36 <= old.y && old.y <= 36);
      bool rightRegion = (24 <= old.x && old.x <= 72 && -36 <= old.y && old.y <= 36);
      bool topRegion = (-60 <= old.x && old.x <= 60 && 48 <= old.y && old.y <= 72);
      bool bottomRegion = (-60 <= old.x && old.x <= 60 && -72 <= old.y && old.y <= -48);

      // Calculate how close we are to each of the four cardinal directions
      float leftAngle = normalizeAngle(180 - old.deg);
      float upAngle = normalizeAngle(90 - old.deg);
      float rightAngle = normalizeAngle(0 - old.deg);
      float downAngle = normalizeAngle(-90 - old.deg);

      // Check if we are within 30 degrees of any of the four cardinal directions
      bool facingLeft = fabs(leftAngle) <= 30;
      bool facingUp = fabs(upAngle) <= 30;
      bool facingRight = fabs(rightAngle) <= 30;
      bool facingDown = fabs(downAngle) <= 30;

      // Update either x or y if we are close enough to a wall and if we are somewhat perpendicular
      if (leftRegion && facingUp && leftDistance <= 48) {
        x = -72 + leftDistance * cos(toRadians(upAngle));
      }
      else if (leftRegion && facingDown && rightDistance <= 48) {
        x = -72 + rightDistance * cos(toRadians(downAngle));
      }
      else if (rightRegion && facingUp && rightDistance <= 48) {
        x = 72 - rightDistance * cos(toRadians(upAngle));
      }
      else if (rightRegion && facingDown && leftDistance <= 48) {
        x = 72 - leftDistance * cos(toRadians(downAngle));
      }
      else if (topRegion && facingLeft && rightDistance <= 24) {
        y = 72 - rightDistance * cos(toRadians(leftAngle));
      }
      else if (topRegion && facingRight && leftDistance <= 24) {
        y = 72 - leftDistance * cos(toRadians(rightAngle));
      }
      else if (bottomRegion && facingLeft && leftDistance <= 24) {
        y = -72 + leftDistance * cos(toRadians(leftAngle));
      }
      else if (bottomRegion && facingRight && rightDistance <= 24) {
        y = -72 + rightDistance * cos(toRadians(rightAngle));
      }

      Pose current = Pose(x, y, th);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateDistance());
    }
};


// Use distance sensors to estimate x and y when we are in one of four regions
// and also use spaced out distance measurements for angle estimation
// We must be driving straight in order to update angle using distance sensors
class DistanceAngleEstimator : public Estimator {
  public:
    int BUFFER_SIZE = 20; // The number of readings between distance measurements
    std::queue<Pose> odometryIMUPoses; // Store up to BUFFER_SIZE odometryIMU poses
    std::queue<float> leftDistances; // Store up to BUFFER_SIZE left distance readings
    std::queue<float> rightDistances; // Store up to BUFFER_SIZE right distance readings

    // Store a pointer to an OdometryIMUEstimator so we can use odometryIMU helper functions
    std::shared_ptr<OdometryIMUEstimator> odometryIMUEstimator;

    DistanceAngleEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Save a pointer to a new OdometryIMUEstimator
      this->odometryIMUEstimator = std::make_shared<OdometryIMUEstimator>(robot);
    }

    // Helper function to estimate a new pose based on odometry, IMU, left and right distance sensors
    Pose estimateDistanceAngle() {
      // Start with the odometry IMU pose as our baseline
      Pose old = this->odometryIMUEstimator->estimateOdometryIMU();
      float x = old.x;
      float y = old.y;
      float th = old.deg;

      // Get the latest pose estimate based on odometry and IMU
      //Pose odometryIMUPose = this->odometryIMUEstimator->estimateOdometryIMU();
      // Read data from left and right distance sensors
      float leftDistance = this->robot->getLeftDistance();
      float rightDistance = this->robot->getRightDistance();

      // Add an odometryIMU pose to the queue
      this->odometryIMUPoses.push(old);
      while(this->BUFFER_SIZE < this->odometryIMUPoses.size()) {
        // Remove items from the queue if it is larger than BUFFER_SIZE
        this->odometryIMUPoses.pop();
      }

      // Add a left distance reading to the queue
      this->leftDistances.push(leftDistance);
      while (this->BUFFER_SIZE < this->leftDistances.size()) {
        // Remove items from the queue if it is larger than BUFFER_SIZE
        this->leftDistances.pop();
      }

      // Add a right distance reading to the queue
      this->rightDistances.push(rightDistance);
      while (this->BUFFER_SIZE < this->rightDistances.size()) {
        // Remove items from the queue if it is larger than BUFFER_SIZE
        this->rightDistances.pop();
      }

      // Calculate the offset from the oldest and newest pose in the queue
      Pose odometryIMUOffset = relativeTransform(odometryIMUPoses.back(), odometryIMUPoses.front());
      // L is the straight distance between poses
      float L = odometryIMUOffset.x;
      // Determine if we were driving straight from the oldest to the newest pose
      bool drivingStraight = fabs(odometryIMUOffset.y) < 1 && fabs(odometryIMUOffset.deg) < 2;

      // Get the oldest and newest reading from each distance sensor queue
      float leftD1 = this->leftDistances.back();
      float leftD2 = this->leftDistances.front();
      float rightD1 = this->rightDistances.back();
      float rightD2 = this->rightDistances.front();

      // Check if we have taken BUFFER_SIZE measurements
      bool filledBuffer = (
        odometryIMUPoses.size() == this->BUFFER_SIZE &&
        leftDistances.size() == this->BUFFER_SIZE &&
        rightDistances.size() == this->BUFFER_SIZE
      );

      // Calculate how close we are to each of the four cardinal directions
      bool leftRegion = (-72 <= old.x && old.x <= -24 && -36 <= old.y && old.y <= 36);
      bool rightRegion = (24 <= old.x && old.x <= 72 && -36 <= old.y && old.y <= 36);
      bool topRegion = (-60 <= old.x && old.x <= 60 && 48 <= old.y && old.y <= 72);
      bool bottomRegion = (-60 <= old.x && old.x <= 60 && -72 <= old.y && old.y <= -48);

      // Calculate how close we are to each of the four cardinal directions
      float leftAngle = normalizeAngle(180 - old.deg);
      float upAngle = normalizeAngle(90 - old.deg);
      float rightAngle = normalizeAngle(0 - old.deg);
      float downAngle = normalizeAngle(-90 - old.deg);

      // Check if we are within 30 degrees of any of the four cardinal directions
      bool facingLeft = fabs(leftAngle) <= 30;
      bool facingUp = fabs(upAngle) <= 30;
      bool facingRight = fabs(rightAngle) <= 30;
      bool facingDown = fabs(downAngle) <= 30;

      // Update either x or y if we are close enough to a wall and if we are somewhat perpendicular
      // Also update th if we have taken enough measurements and we have been driving straight
      if (leftRegion && facingUp && leftDistance <= 48) {
        x = -72 + leftDistance * cos(toRadians(upAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 90 + atan2((leftD1 - leftD2), L);
        }
      }
      else if (leftRegion && facingDown && rightDistance <= 48) {
        x = -72 + rightDistance * cos(toRadians(downAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = -90 + atan2((rightD2 - rightD1), L);
        }
      }
      else if (rightRegion && facingUp && rightDistance <= 48) {
        x = 72 - rightDistance * cos(toRadians(upAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 90 + atan2((rightD2 - rightD1), L);
        }
      }
      else if (rightRegion && facingDown && leftDistance <= 48) {
        x = 72 - leftDistance * cos(toRadians(downAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = -90 + atan2((leftD1 - leftD2), L);
        }
      }
      else if (topRegion && facingLeft && rightDistance <= 24) {
        y = 72 - rightDistance * cos(toRadians(leftAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 180 + atan2((rightD2 - rightD1), L);
        }
      }
      else if (topRegion && facingRight && leftDistance <= 24) {
        y = 72 - leftDistance * cos(toRadians(rightAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 0 + atan2((leftD1 - leftD2), L);
        }
      }
      else if (bottomRegion && facingLeft && leftDistance <= 24) {
        y = -72 + leftDistance * cos(toRadians(leftAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 180 + atan2((leftD1 - leftD2), L);
        }
      }
      else if (bottomRegion && facingRight && rightDistance <= 24) {
        y = -72 + rightDistance * cos(toRadians(rightAngle));
        if (filledBuffer && drivingStraight && L != 0) {
          th = 0 + atan2((rightD2 - rightD1), L);
        }
      }

      Pose current = Pose(x, y, th);
      this->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateDistanceAngle());
    }
};


// Fuse data from everything so far for full pose estimation
class FusionEstimator : public Estimator {
  public:
    // Percent to use for each odometry+IMU and distance sensor estimation, should add up to 1.0
    float PERCENT_ODOMETRY_IMU = 0.95;
    float PERCENT_DISTANCE = 0.05;

    // Store a pointer to a DistanceAngleEstimator so we can use distanceAngle helper functions
    std::shared_ptr<DistanceAngleEstimator> distanceAngleEstimator;

    FusionEstimator(std::shared_ptr<Robot> robot) {
      // Initialize the estimator and save a pointer to the robot
      this->robot = robot;
      this->current = this->robot->current;
      // Save a pointer to a new DistanceAngleEstimator
      this->distanceAngleEstimator = std::make_shared<DistanceAngleEstimator>(robot);
    }

    // Change the ratio of percent to use for odometry+IMU and distance sensor estimation
    void setRatio(float percentOdometryIMU, float percentDistance) {
      this->PERCENT_ODOMETRY_IMU = percentOdometryIMU;
      this->PERCENT_DISTANCE = percentDistance;
    }

    // Helper function to estimate a new pose based on odometry, IMU, and distance sensors
    Pose estimateFusion() {
      // Get the estimated pose from the DistanceAngleEstimator
      Pose distanceAnglePose = this->distanceAngleEstimator->estimateDistanceAngle();
      // Get the latest estimated odometryIMU pose from the DistanceAngleEstimator buffer
      Pose odometryIMUPose = this->distanceAngleEstimator->odometryIMUPoses.back();

      // Start with the odometryIMU pose as our baseline
      float x = odometryIMUPose.x;
      float y = odometryIMUPose.y;
      float th = odometryIMUPose.deg;

      // Only update x, y, or th if the distance readings produced any changes
      // If so, use a certain percent from odometryIMU and a certain percent from distance sensors
      // This is a simple form of sensor fusion
      if (distanceAnglePose.x != this->robot->current.x) {
        x = (odometryIMUPose.x * this->PERCENT_ODOMETRY_IMU) + (distanceAnglePose.x * this->PERCENT_DISTANCE);
      }

      if (distanceAnglePose.y != this->robot->current.y) {
        y = (odometryIMUPose.y * this->PERCENT_ODOMETRY_IMU) + (distanceAnglePose.y * this->PERCENT_DISTANCE);
      }

      if (distanceAnglePose.deg != this->robot->current.deg) {
        th = (odometryIMUPose.deg * this->PERCENT_ODOMETRY_IMU) + (distanceAnglePose.deg * this->PERCENT_DISTANCE);
      }

      Pose current = Pose(x, y, th);
      this->current = current;
      this->distanceAngleEstimator->odometryIMUEstimator->odometryEstimator->current = current;
      return current;
    }

    // Override the update function and update our current pose
    void update() override {
      this->robot->setCurrent(estimateFusion());
    }
};


#endif