#ifndef GENERATORS_H
#define GENERATORS_H

#include <memory>


// This is a virtual base class for Generators
// Essentially a template that we should follow in order to implement other Generators
class Generator {
  public:
    // Store a pointer to a robot so we can read data from the robot
    std::shared_ptr<Robot> robot;

    // We need to include this so all of our Generators are destroyed properly
    virtual ~Generator() = default;

    // This is the main function that we need to implement for each Generator
    // Return a vector of Poses that represent a path to follow
    virtual std::vector<Pose> getPoints() = 0;
};


// Generate a set of points for driving in a straight line
class StraightGenerator: public Generator {
  public:
    float targetDistance = 0;

    StraightGenerator(std::shared_ptr<Robot> robot, float targetDistance) {
      // Initialize the generator and save a pointer to the robot
      this->robot = robot;
      // Save the target distance
      this->targetDistance = targetDistance;
    }

    // Override the getPoints function and return a vector of points
    std::vector<Pose> getPoints() override {
      std::vector<Pose> points;
      // Robot's current pose is the starting point
      Pose current = this->robot->current;
      float totalDistance = 0;

      float speed = robot->getMaxSpeed();
      // If our target is backwards, calculate with negative speed
      if (targetDistance < 0) {
        speed = -speed;
      }

      // Add points until cumulative distance reaches target distance
      while (fabs(totalDistance) <= fabs(this->targetDistance)) {
        // Add new pose based on the current starting location
        Pose newPose = absoluteTransform(current, Pose(totalDistance, 0, 0));
        points.push_back(newPose);
        // Calculate a small distance for one control interval
        totalDistance += speed * this->robot->INTERVAL_TIME;
      }

      return points;
    }
};


// Generate a set of points for turning in place
class TurnGenerator: public Generator {
  public:
    float targetAngle = 0;

    TurnGenerator(std::shared_ptr<Robot> robot, float targetAngle) {
      // Initialize the generator and save a pointer to the robot
      this->robot = robot;
      // Save the target angle
      this->targetAngle = targetAngle;
    }

    // Override the getPoints function and return a vector of points
    std::vector<Pose> getPoints() override {
      std::vector<Pose> points;
      // Robot's current pose is the starting point
      Pose current = this->robot->current;
      float totalAngle = 0;

      // If our target is clockwise, calculate with negative speed
      float speed = robot->getMaxSpeed();
      if (targetAngle < 0) {
        speed = -speed;
      }

      // Add points until cumulative angle reaches target angle
      while (fabs(totalAngle) <= fabs(this->targetAngle)) {
        // Add new pose based on the current starting location
        Pose newPose = absoluteTransform(current, Pose(0, 0, totalAngle));
        points.push_back(newPose);
        // Calculate a small angle for one control interval
        totalAngle += toDegrees((2 * speed * this->robot->INTERVAL_TIME) / this->robot->TRACK_WIDTH);
      }

      return points;
    }
};


// Generate a set of points to reach any absolute pose
class AbsoluteCubicGenerator: public Generator {
  public:
    Pose target = Pose(0, 0, 0);
    bool reverse = false;

    AbsoluteCubicGenerator(std::shared_ptr<Robot> robot, Pose target, bool reverse = false) {
      // Initialize the generator and save a pointer to the robot
      this->robot = robot;
      // Save the target pose
      this->target = target;
      // Optional argument for executing the path backwards
      this->reverse = reverse;

      if (this->reverse) {
        // For reverse motions, calculate as if the target is backwards
        this->target = absoluteTransform(this->target, Pose(0, 0, 180));
      }
    }

  // Override the getPoints function and return a vector of points
  std::vector<Pose> getPoints() override {
    std::vector<Pose> points;

    // Robot's current pose is the starting point
    Pose current = this->robot->current;
    if (this->reverse) {
      // For reverse motions, calculate as if we start facing backwards
      current = absoluteTransform(this->robot->current, Pose(0, 0, 180));
    }
    
    float speed = this->robot->getMaxSpeed();
    float totalTime = 0;

    // Approximate the total time based on distance to target
    float targetDistance = sqrt(pow(current.x - this->target.x, 2) + pow(current.y - this->target.y, 2));
    float targetTime = 1.5 * targetDistance / speed;

    // Calculate coefficients for cubic spline to reach target pose from start pose
    float v0x = speed * cos(current.rad);
    float v0y = speed * sin(current.rad);
    float v1x = speed * cos(target.rad);
    float v1y = speed * sin(target.rad);

    float ax = current.x;
    float bx = v0x;
    float cx = (3 * (target.x - current.x) - targetTime * (2 * v0x + v1x)) / pow(targetTime, 2);
    float dx = (2 * (current.x - target.x) + targetTime * (v0x + v1x)) / pow(targetTime, 3);

    float ay = current.y;
    float by = v0y;
    float cy = (3 * (target.y - current.y) - targetTime * (2 * v0y + v1y)) / pow(targetTime, 2);
    float dy = (2 * (current.y - target.y) + targetTime * (v0y + v1y)) / pow(targetTime, 3);

    while (totalTime <= targetTime) {
      // Calculate pose of cubic spline for each time point
      float x = ax + bx * totalTime + cx * pow(totalTime, 2) + dx * pow(totalTime, 3);
      float y = ay + by * totalTime + cy * pow(totalTime, 2) + dy * pow(totalTime, 3);
      float vx = bx + 2 * cx * totalTime + 3 * dx * pow(totalTime, 2);
      float vy = by + 2 * cy * totalTime + 3 * dy * pow(totalTime, 2);
      float th = toDegrees(atan2(vy, vx));

      if (this->reverse) {
        // For reverse motions, add a backwards pose along the cubic spline
        points.push_back(Pose(x, y, normalizeAngle(th + 180)));
      }
      else {
        // Add new pose along the cubic spline
        points.push_back(Pose(x, y, th));
      }

      totalTime += this->robot->INTERVAL_TIME;
    }

    return points;
  }
};


// Generate a set of points to reach any relative pose
class RelativeCubicGenerator : public AbsoluteCubicGenerator {
  public:

    RelativeCubicGenerator(std::shared_ptr<Robot> robot, Pose target, bool reverse = false) :
      AbsoluteCubicGenerator(robot, target) {
      // Initialize the generator and save a pointer to the robot
      this->robot = robot;
      // Given a relative target, save the target as an absolute pose based on the robot's current pose
      this->target = absoluteTransform(this->robot->current, target);
      // Optional argument for executing the path backwards
      this->reverse = reverse;
      
      if (this->reverse) {
        // For reverse motions, calculate as if the target is backwards
        this->target = absoluteTransform(this->target, Pose(0, 0, 180));
      }
    }
};


#endif