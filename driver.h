#ifndef DRIVER_H
#define DRIVER_H

#include <memory>


class Driver {
  public:
    // Store pointers to all components
    std::shared_ptr<Robot> robot;
    std::shared_ptr<Estimator> estimator;
    std::shared_ptr<Generator> generator;
    std::shared_ptr<Controller> controller;
    std::shared_ptr<Controller> turnController;
    // Store the trajectory points to be executed
    std::vector<Pose> points;

    Driver() = default;

    // Initialize the driver with the robot start location
    Driver(Pose start) {
      // Create a robot and store in a pointer
      this->robot = std::make_shared<Robot>();
      this->robot->setCurrent(start);
      
      // Set default estimator and controllers
      setEstimator<DirectEstimator>();
      setController<FeedforwardController>();
      setTurnController<FeedforwardController>();
    }

    // Set the estimator and pass any needed arguments
    template<typename T, typename... Args>
    void setEstimator(Args&&... args) {
      this->estimator = std::make_shared<T>(this->robot, std::forward<Args>(args)...);
    }

    // Set the generator and pass any needed arguments
    template<typename T, typename... Args>
    void setGenerator(Args&&... args) {
      this->generator = std::make_shared<T>(this->robot, std::forward<Args>(args)...);
    }

    // Set the controller and pass any needed arguments
    template<typename T, typename... Args>
    void setController(Args&&... args) {
      this->controller = std::make_shared<T>(this->robot, std::forward<Args>(args)...);
    }
    
    // Set the turn controller and pass any needed arguments
    template<typename T, typename... Args>
    void setTurnController(Args&&... args) {
      this->turnController = std::make_shared<T>(this->robot, std::forward<Args>(args)...);
    }

    // Main control loop - drive through all points that are currently stored
    void run(bool print = false) {
      // Make sure our current pose is the beginning of our trajectory
      this->robot->setCurrent(this->points.front());
      
      for (int i = 1; i < this->points.size(); i++) {
        // Get control time based on trajectory progress
        float t = this->robot->getTime(i / float(this->points.size()));
  
        // Update the robot's pose estimate
        this->estimator->update();
        
        Pose offset = relativeTransform(this->points[i-1], this->points[i]);
        if (offset.x == 0 && offset.y == 0 && offset.deg != 0) {
          // Use the turn controller to move the robot if we are turning in place
          i = this->turnController->sendCommand(this->points, i, t);
        }
        else {
          // Use the controller to move the robot
          i = this->controller->sendCommand(this->points, i, t);
        }
        
        // Option to print pose estimate while driving
        if (print) {
          printCurrent();
        }

        wait(t, sec);
      }

      // Reset points when trajectory is completed
      this->points = {};
      robot->stopAll();
      // Let the robot finish any motion and perform final update
      wait(0.2, sec);
      this->estimator->update();
    }
    
    void addPoints() {
      // Use the current generator to create a new set of points
      std::vector<Pose> newPoints = this->generator->getPoints();
      // Add new points to current points
      this->points.insert(this->points.end(), newPoints.begin(), newPoints.end());
      // Update the robot pose for subsequent trajectories
      this->robot->setCurrent(this->points.back());
    }
    
    // Print all the points that are currently stored
    void printPoints() {
      for (int i = 0; i < this->points.size(); i++) {
        points[i].print();
        wait(0.01, sec);
      }
    }
    
    // Print the robot's current pose
    void printCurrent() {
      this->robot->current.print();
    }

    // Add a straight trajectory to stored points
    void straight(float distance) {
      setGenerator<StraightGenerator>(distance);
      addPoints();
    }

    // Add a turning trajectory to stored points
    void turn(float angle) {
      setGenerator<TurnGenerator>(angle);
      addPoints();
    }

    // Add an absolute target trajectory to stored points
    void absolute(float x, float y, float deg, bool reverse = false) {
      setGenerator<AbsoluteCubicGenerator>(Pose(x, y, deg), reverse);
      addPoints();
    }

    // Add a relative target trajectory to stored points
    void relative(float x, float y, float deg, bool reverse = false) {
      setGenerator<RelativeCubicGenerator>(Pose(x, y, deg), reverse);
      addPoints();
    }
};


#endif