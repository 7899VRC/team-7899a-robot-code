#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <memory>


// This is a virtual base class for Controllers
// Essentially a template that we should follow in order to implement other Controllers
class Controller {
  public:
    // Store a pointer to a robot so we can read data from the robot
    std::shared_ptr<Robot> robot;

    // We need to include this so all of our Controllers are destroyed properly
    virtual ~Controller() = default;

    // This is the main function that we need to implement for each Controller
    // Given a trajectory, index of next point, and time duration, send a command for the robot to execute
    virtual int sendCommand(std::vector<Pose> points, int i, float t) = 0;
};


// Control based on expected velocities only
class FeedforwardController : public Controller {
  public:

    FeedforwardController(std::shared_ptr<Robot> robot) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
    }

    // Helper function to create a command based on trajectory
    Command getFeedforwardCommand(std::vector<Pose> points, int i, float t) {
      Command command;
  
      // Make sure we have at least two points to get an offset
      if (0 < i) {
        // Calculate the relative offset between two points in the trajectory
        Pose offset = relativeTransform(points[i-1], points[i]);
        // Calculate linear and angular velocity based on arc motion
        command = this->robot->getPoseCommand(offset, t);
      }
      return command;
    }

    // Create a command and send to the robot
    int sendCommand(std::vector<Pose> points, int i, float t) override {
      this->robot->execute(getFeedforwardCommand(points, i, t));
      return i;
    }
};


// Control based on PID error to trajectory
class PIDController : public Controller {
  public:
    // Linear PID
    float KPV = 1;
    float KIV = 0;
    float KDV = 0;

    // Angular PID
    float KPW = 1;
    float KIW = 0;
    float KDW = 0;

    // Integral limits
    float dMaxIntegral = 1000;
    float thMaxIntegral = 50;

    // Components for integral and derivative
    float dIntegral = 0;
    float thIntegral = 0;
    float dPrevious = 0;
    float thPrevious = 0;

    PIDController(std::shared_ptr<Robot> robot) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
    }

    PIDController(std::shared_ptr<Robot> robot, float KPV, float KIV, float KDV, float KPW, float KIW, float KDW) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
      // Override default PID values
      this->KPV = KPV;
      this->KIV = KIV;
      this->KDV = KDV;
      this->KPW = KPW;
      this->KIW = KIW;
      this->KDW = KDW;
    }

    // Helper function to create a command based on trajectory
    Command getPIDCommand(std::vector<Pose> points, int i, float t) { 
      Pose current = this->robot->current;
      Pose target = points[i];
      
      // Calculate the offset between current pose and next trajectory point
      Pose offset = relativeTransform(current, target);

      // Linear velocity corrects linear error along the path
      float dError = offset.x;
      // Angular velocity corrects lateral error by pointing towards the path
      float thError = offset.y;

      // Add up error for integral terms
      this->dIntegral = fmax(fmin(this->dIntegral + dError, this->dMaxIntegral), -this->dMaxIntegral);
      this->thIntegral = fmax(fmin(this->thIntegral + thError, this->thMaxIntegral), -this->thMaxIntegral);

      // PID control formulas for linear and angular velocity
      float v = (this->KPV * dError) + (this->KIV * this->dIntegral) + (this->KDV * (dError - this->dPrevious) / t);
      float w = (this->KPW * thError) + (this->KIW * this->thIntegral) + (this->KDW * (thError - this->thPrevious) / t);

      // Save the current error for the next derivative terms
      this->dPrevious = dError;
      this->thPrevious = thError;

      return Command(v, w);
    }

    // Create a command and send to the robot
    int sendCommand(std::vector<Pose> points, int i, float t) override {
      this->robot->execute(getPIDCommand(points, i, t));
      return i;
    }
};


// Control based on PID error for turning in place
class PIDTurnController : public PIDController {
  public:
    // Angular PID
    float KPW = 1;
    float KIW = 0;
    float KDW = 0;

    PIDTurnController(std::shared_ptr<Robot> robot) :
      PIDController(robot) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
    }

    PIDTurnController(std::shared_ptr<Robot> robot, float KPW, float KIW, float KDW) :
      PIDController(robot, 0, 0, 0, KPW, KIW, KDW) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
      // Override default PID values
      this->KPW = KPW;
      this->KIW = KIW;
      this->KDW = KDW;
    }

    // Helper function to create a command based on trajectory
    Command getPIDTurnCommand(std::vector<Pose> points, int i, float t) {
      Pose current = this->robot->current;
      Pose target = points[i];

      // Calculate the error between current angle and next trajectory angle
      float thError = toRadians(normalizeAngle(target.deg - current.deg));

      // Add up error for integral term
      this->thIntegral = fmax(fmin(this->thIntegral + thError, this->thMaxIntegral), -this->thMaxIntegral);

      // PID control formula for angular velocity
      float v = 0;
      float w = (this->KPW * thError) + (this->KIW * this->thIntegral) + (this->KDW * (thError - this->thPrevious) / t);

      // Save the current error for the next derivative terms
      this->thPrevious = thError;

      return Command(v, w);
    }

    // Create a command and send to the robot
    int sendCommand(std::vector<Pose> points, int i, float t) override {
      this->robot->execute(getPIDTurnCommand(points, i, t));
      return i;
    }
};


// Control based on expected velocities and PID error to trajectory
class FeedforwardPIDController : public Controller {
  public:
    // Linear PID
    float KPV = 0.1;
    float KIV = 0;
    float KDV = 0;

    // Angular PID
    float KPW = 0.1;
    float KIW = 0;
    float KDW = 0;

    // Store a pointer to a FeedforwardController so we can use feedforward helper functions
    std::shared_ptr<FeedforwardController> feedforwardController;
    // Store a pointer to a PIDController so we can use PID helper functions
    std::shared_ptr<PIDController> pidController;

    FeedforwardPIDController(std::shared_ptr<Robot> robot) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
      // Save a pointer to a new FeedforwardController
      this->feedforwardController = std::make_shared<FeedforwardController>(robot);
      // Save a pointer to a new PIDController
      this->pidController = std::make_shared<PIDController>(
        robot, this->KPV, this->KIV, this->KDV, this->KPW, this->KIW, this->KDW);
    }

    FeedforwardPIDController(
      std::shared_ptr<Robot> robot, float KPV, float KIV, float KDV, float KPW, float KIW, float KDW) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
      // Override default PID values
      this->KPV = KPV;
      this->KIV = KIV;
      this->KDV = KDV;
      this->KPW = KPW;
      this->KIW = KIW;
      this->KDW = KDW;
      // Save a pointer to a new FeedforwardController
      this->feedforwardController = std::make_shared<FeedforwardController>(robot);
      // Save a pointer to a new PIDController
      this->pidController = std::make_shared<PIDController>(
        robot, this->KPV, this->KIV, this->KDV, this->KPW, this->KIW, this->KDW);
    }

    // Helper function to create a command based on trajectory
    Command getFeedforwardPIDCommand(std::vector<Pose> points, int i, float t) {
      Command feedforwardCommand = this->feedforwardController->getFeedforwardCommand(points, i, t);
      Command pidCommand = this->pidController->getPIDCommand(points, i, t);

      // Combine the commands from the feedforward and PID controllers
      float v = feedforwardCommand.v + pidCommand.v;
      float w = feedforwardCommand.w + pidCommand.w;
  
      return Command(v, w);
    }

    // Create a command and send to the robot
    int sendCommand(std::vector<Pose> points, int i, float t) override {
      this->robot->execute(getFeedforwardPIDCommand(points, i, t));
      return i;
    }
};


// Control based on pure pursuit error to trajectory
class PurePursuitController : public Controller {
  public:
    int LOOKAHEAD = 30; // Number of points to look ahead in the trajectory
    float ACCURACY = 2; // Stop if we are within this distance of the target

    int closestIndex = 0; // Most recent closest index in trajectory
    int timeIndex = 0; // Index closest to current expected position
    bool finished = true; // Have we reached the target

    PurePursuitController(std::shared_ptr<Robot> robot) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
    }

    PurePursuitController(std::shared_ptr<Robot> robot, int lookahead, float accuracy) {
      // Initialize the controller and save a pointer to the robot
      this->robot = robot;
      // Override lookahead and accuracy
      this->LOOKAHEAD = lookahead;
      this->ACCURACY = accuracy;

    }

    // Helper function to create a command based on trajectory
    Command getPurePursuitCommand(std::vector<Pose> points, int i, float t) {
      // Reset finished status if we are starting a new trajectory
      if (i == 1) {
        this->finished = false;
        this->closestIndex = 0;
      }

      Pose current = this->robot->current;
      float minimumDistance = 1;

      // Search the trajectory for the point that is closest to the robot's current pose
      for (int index = this->closestIndex; index < points.size(); index++) {
        Pose closestPoint = points[index];
        float distance = calculateDistance(current, closestPoint);
        if (distance < minimumDistance) {
          minimumDistance = distance;
          this->closestIndex = index;
        }
      }

      // Set the target to the closest point plus lookahead
      int targetIndex = std::min(std::max(this->closestIndex, i) + this->LOOKAHEAD, int(points.size() - 1));
      Pose target = points[targetIndex];
      Pose offset = relativeTransform(current, target);

      // Calculate curvature using offset
      float L2 = pow(offset.x, 2) + pow(offset.y, 2);
      float curvature = 0;
      if (0.0001 < L2) {
        curvature = (2.0 * offset.y / L2);
      }

      // Check if we need to scale time for speed ramping based on trajectory progress
      this->timeIndex = std::max(std::min(this->closestIndex, targetIndex), i);
      t = robot->getTime(this->timeIndex / float(points.size()));
      // Set linear and angular velocity
      float v = this->robot->getMaxSpeed() * (this->robot->INTERVAL_TIME / t);
      // If our target is backwards, calculate with negative speed
      if (offset.x < 0) {
        v = -v;
      }
      float w = v * curvature;

      float distanceToEnd = calculateDistance(current, points.back());

      // Check the distance to see if we have reached the target
      if (distanceToEnd < this->ACCURACY) {
        this->finished = true;
        this->robot->stopAll();
      }

      // Only send velocities if we have not yet reached the target
      if (this->finished) {
        return Command(0, 0);
      }
      else {
        return Command(v, w);
      }
    }

    int sendCommand(std::vector<Pose> points, int i, float t) override {
      this->robot->execute(getPurePursuitCommand(points, i, t));
      return this->timeIndex;
    }
};


#endif