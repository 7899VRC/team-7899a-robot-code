#ifndef EXAMPLES_H
#define EXAMPLES_H


#pragma region Lab1Examples
void example1_1() {
  Robot robot = Robot();

  // Spin each motor to see if orientation and port is correct
  robot.LEFT_MOTORS[0].spin(forward, 20, pct);
  wait(1, sec);
  robot.LEFT_MOTORS[0].stop(brake);

  robot.RIGHT_MOTORS[0].spin(forward, 20, pct);
  wait(1, sec);
  robot.RIGHT_MOTORS[0].stop(brake);
}


void example1_2() {
  Robot robot = Robot();

  // Test out individual robot control functions
  robot.leftDrive(30);
  robot.rightDrive(30);
  wait(1, sec);
  robot.stopAll();

  robot.leftDrive(-30);
  robot.rightDrive(-30);
  wait(1, sec);
  robot.stopAll();

  robot.leftDrive(-30);
  robot.rightDrive(30);
  wait(1, sec);
  robot.stopAll();

  robot.leftDrive(30);
  robot.rightDrive(-30);
  wait(1, sec);
  robot.stopAll();
}


void example1_3() {
  Robot robot = Robot();

  // Test out driving to any position
  // This function will not use the target angle
  // Make sure to give enough time so we don't hit the speed limit
  float t = 5;
  robot.driveToPose(Pose(20, 10, 0), t);
  wait(t, sec);
  robot.stopAll();
}


void example1_4() {
  Robot robot = Robot();
  
  // Drive straight and turn in place
  robot.straight(20);
  robot.stopAll();
  
  robot.turn(90);
  robot.stopAll();

  robot.turn(-90);
  robot.stopAll();

  robot.straight(20);
  robot.stopAll();  
}


void example1_5() {
  Robot robot = Robot();

  // Enable ramp up and ramp down
  robot.setRampUp(true);
  robot.setRampDown(true);
  
  // Drive straight and turn in place with speed ramping
  robot.straightRamp(20);
  robot.stopAll();
  
  robot.turnRamp(90);
  robot.stopAll();

  robot.turnRamp(-90);
  robot.stopAll();

  robot.straightRamp(-20);
  robot.stopAll();
}
#pragma endregion


#pragma region Lab2Examples
// Try out direct estimation
// We need to set the latest offset for each motion
void example2_1() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  DirectEstimator estimator = DirectEstimator(robot);
  
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->straightRamp(20);
  robot->setOffset(Pose(20, 0, 0));
  robot->stopAll();
  wait(1, sec);
  estimator.update();
  
  robot->turnRamp(-90);
  robot->setOffset(Pose(0, 0, -90));
  robot->stopAll();
  wait(1, sec);
  estimator.update();
  
  robot->straightRamp(20);
  robot->setOffset(Pose(20, 0, 0));
  robot->stopAll();
  wait(1, sec);
  estimator.update();
  
  robot->turnRamp(90);
  robot->setOffset(Pose(0, 0, 90));
  robot->stopAll();
  wait(1, sec);
  estimator.update();
  
  robot->straightRamp(20);
  robot->setOffset(Pose(20, 0, 0));
  robot->stopAll();
  wait(1, sec);
  estimator.update();

  robot->current.printBrain();
  wait(100, sec);
}


// Try using encoders for straight motions and IMU for turns
void example2_2() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  EncoderEstimator encoderEstimator = EncoderEstimator(robot);
  IMUEstimator imuEstimator = IMUEstimator(robot);
  
  robot->setRampUp(true);
  robot->setRampDown(true);

  encoderEstimator = EncoderEstimator(robot);
  robot->straightRamp(20);
  robot->stopAll();
  wait(1, sec);
  encoderEstimator.update();
  
  imuEstimator = IMUEstimator(robot);
  robot->turnRamp(-90);
  robot->stopAll();
  wait(1, sec);
  imuEstimator.update();
  
  encoderEstimator = EncoderEstimator(robot);
  robot->straightRamp(20);
  robot->stopAll();
  wait(1, sec);
  encoderEstimator.update();
  
  imuEstimator = IMUEstimator(robot);
  robot->turnRamp(90);
  robot->stopAll();
  wait(1, sec);
  imuEstimator.update();
  
  encoderEstimator = EncoderEstimator(robot);
  robot->straightRamp(20);
  robot->stopAll();
  wait(1, sec);
  encoderEstimator.update();

  robot->current.printBrain();
  wait(100, sec);
}


// Try out odometry estimation with some curved motions
void example2_3() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  OdometryEstimator estimator = OdometryEstimator(robot);

  robot->leftDrive(25);
  robot->rightDrive(20);
  wait(2, sec);
  estimator.update();
  
  robot->leftDrive(20);
  robot->rightDrive(30);
  wait(1, sec);
  estimator.update();
  
  robot->current.printBrain();
  robot->stopAll();
  wait(100, sec);
}


// Try out odometry estimation improved with IMU
// Drive and update in a fast control loop
void example2_4() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  OdometryIMUEstimator estimator = OdometryIMUEstimator(robot);

  for (int i = 0; i < 50; i++) {
    robot->leftDrive(30);
    robot->rightDrive(40);
    estimator.update();
    wait(robot->INTERVAL_TIME, sec);
  }

  robot->current.printBrain();
  robot->stopAll();
  wait(100, sec);
}


// Try out basic distance estimation
// We need to set a valid starting pose to use this estimator
void example2_5() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  robot->setCurrent(Pose(30, 0, 90));

  BasicDistanceEstimator estimator = BasicDistanceEstimator(robot);

  for (int i = 0; i < 100; i++) {
    estimator.update();
    robot->current.printBrain();
    wait(0.5, sec);
  }
}


// Try out distance estimation
// We have a wider range of valid starting poses that we can start with
void example2_6() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  robot->setCurrent(Pose(30, 0, 100));

  DistanceEstimator estimator = DistanceEstimator(robot);

  for (int i = 0; i < 100; i++) {
    estimator.update();
    robot->current.printBrain();
    wait(0.5, sec);
  }
}


// Try fusing every method of estimation
// Drive straight next to a wall and see if our estimate improves
void example2_7() {
  std::shared_ptr<Robot> robot = std::make_shared<Robot>();
  robot->setCurrent(Pose(30, 0, 100));

  FusionEstimator estimator = FusionEstimator(robot);

  for (int i = 0; i < 50; i++) {
    robot->leftDrive(20);
    robot->rightDrive(20);
    estimator.update();
    wait(robot->INTERVAL_TIME, sec);
    robot->current.print();
  }

  robot->current.printBrain();
  robot->stopAll();
  wait(100, sec);
}
#pragma endregion


#pragma region Lab3Examples
// Try out a straight trajectory
void example3_1() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;

  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the generator
  robot->setCurrent(Pose(50, 0, 90));

  // Create a straight trajectory
  generator = std::make_shared<StraightGenerator>(robot, 30);
  points = generator->getPoints();
  
  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
}


// Try out a turn trajectory
void example3_2() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;

  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the generator
  robot->setCurrent(Pose(50, 0, 90));

  // Create a turn trajectory
  generator = std::make_shared<TurnGenerator>(robot, -135);
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
}


// Try out an absolute cubic spline trajectory
void example3_3() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;

  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the generator
  robot->setCurrent(Pose(0, 0, 0));

  // Create a cubic spline trajectory to an absolute target
  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(40, -16, 30));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
}


// Try out an absolute and relative cubic spline trajectory
void example3_4() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;

  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the generator
  robot->setCurrent(Pose(50, 0, 90));

  // Create a cubic spline trajectory to an absolute target
  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(20, -20, 90));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();

  // Set the next pose before creating the next generator
  robot->setCurrent(Pose(20, -20, 90));

  // Create a cubic spline trajectory to a relative target
  generator = std::make_shared<RelativeCubicGenerator>(robot, Pose(20, -10, 0));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    robot->stopAll();
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
}


// Try out an absolute and relative cubic spline trajectory backwards
void example3_5() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;

  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the generator
  robot->setCurrent(Pose(50, 0, 90));

  // Create a cubic spline trajectory to an absolute target
  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(20, -20, -90), true);
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
 
  // Set the next pose before creating the next generator
  robot->setCurrent(Pose(20, -20, -90));

  // Create a cubic spline trajectory to a relative target
  generator = std::make_shared<RelativeCubicGenerator>(robot, Pose(-20, 10, 0), true);
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    current = next;
    wait(t, sec);
  }
  robot->stopAll();
}


// Try out absolute cubic spline trajectories with state estimation
void example3_6() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::vector<Pose> points;
  Pose current;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  // We need to set the current pose before creating the estimator and generator
  robot->setCurrent(Pose(40, 0, 90));

  // We can try driving with different estimators if sensors are set up
  estimator = std::make_shared<OdometryEstimator>(robot);
  //estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  // Create a cubic spline trajectory to an absolute target
  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(50, 20, 120));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    // Update state estimation
    estimator->update();
    robot->current.print();
    current = next;
    wait(t, sec);
  }
  robot->stopAll();


  // Set the next pose before creating the next generator
  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(45, 0, -90));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  // Drive to each point
  current = robot->current;
  for (int i = 0; i < points.size(); i++) {
    Pose next = points[i];
    // Calculate the offset from current to the next point
    Pose offset = relativeTransform(current, next);
    // Time may be stretched if we have speed ramping enabled
    float t = robot->getTime(i / float(points.size()));
    // Drive to the next point
    robot->driveToPose(offset, t);
    // Update state estimation
    estimator->update();
    robot->current.print();
    current = next;
    wait(t, sec);
  }
  robot->stopAll();

  robot->current.printBrain();
  wait(100, sec);
}
#pragma endregion


#pragma region Lab4Examples
void example4_1() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::shared_ptr<Controller> controller;
  std::vector<Pose> points;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->setCurrent(Pose(40, 0, 90));

  //estimator = std::make_shared<OdometryEstimator>(robot);
  estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  //controller = std::make_shared<FeedforwardController>(robot);
  controller = std::make_shared<PIDController>(robot, 1, 0, 0, 1, 0, 0);

  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(60, 0, 90));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  for (int i = 0; i < points.size(); i++) {
    float t = robot->getTime(i / float(points.size()));
  
    estimator->update();
    // Use the controller to send a command for every trajectory point
    controller->sendCommand(points, i, t);
    robot->current.print();

    wait(t, sec);
  }
  robot->stopAll();
}


void example4_2() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::shared_ptr<Controller> controller;
  std::vector<Pose> points;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->setCurrent(Pose(40, 0, 90));

  //estimator = std::make_shared<OdometryEstimator>(robot);
  estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  //controller = std::make_shared<FeedforwardController>(robot);
  controller = std::make_shared<PIDController>(robot, 1, 0, 0, 1, 0, 0);

  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(50, 20, 120));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  for (int i = 0; i < points.size(); i++) {
    float t = robot->getTime(i / float(points.size()));
  
    estimator->update();
    // Use the controller to send a command for every trajectory point
    controller->sendCommand(points, i, t);
    robot->current.print();

    wait(t, sec);
  }
  robot->stopAll();
}


void example4_3() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::shared_ptr<Controller> controller;
  std::vector<Pose> points;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->setCurrent(Pose(40, 0, 90));

  //estimator = std::make_shared<OdometryEstimator>(robot);
  estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  //controller = std::make_shared<FeedforwardController>(robot);
  controller = std::make_shared<PIDTurnController>(robot, 1, 0, 0);

  generator = std::make_shared<TurnGenerator>(robot, -90);
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  for (int i = 0; i < points.size(); i++) {
    float t = robot->getTime(i / float(points.size()));
  
    estimator->update();
    // Use the controller to send a command for every trajectory point
    controller->sendCommand(points, i, t);
    robot->current.print();

    wait(t, sec);
  }
  robot->stopAll();
}


void example4_4() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::shared_ptr<Controller> controller;
  std::vector<Pose> points;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->setCurrent(Pose(40, 0, 90));

  //estimator = std::make_shared<OdometryEstimator>(robot);
  estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  controller = std::make_shared<FeedforwardPIDController>(robot, 0.1, 0, 0, 0.1, 0, 0);

  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(50, 20, 120));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  for (int i = 0; i < points.size(); i++) {
    float t = robot->getTime(i / float(points.size()));
  
    estimator->update();
    // Use the controller to send a command for every trajectory point
    controller->sendCommand(points, i, t);
    robot->current.print();

    wait(t, sec);
  }
  robot->stopAll();
}


void example4_5() {
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Estimator> estimator;
  std::shared_ptr<Generator> generator;
  std::shared_ptr<Controller> controller;
  std::vector<Pose> points;
  
  robot = std::make_shared<Robot>();
  robot->setRampUp(true);
  robot->setRampDown(true);

  robot->setCurrent(Pose(40, 0, 90));

  //estimator = std::make_shared<OdometryEstimator>(robot);
  estimator = std::make_shared<OdometryIMUEstimator>(robot);
  //estimator = std::make_shared<FusionEstimator>(robot);

  controller = std::make_shared<PurePursuitController>(robot, 30, 2);

  generator = std::make_shared<AbsoluteCubicGenerator>(robot, Pose(50, 20, 120));
  points = generator->getPoints();

  // Just print the points so we can see them
  for (int i = 0; i < points.size(); i++) {
    points[i].print();
    wait(0.01, sec);
  }

  for (int i = 0; i < points.size(); i++) {
    float t = robot->getTime(i / float(points.size()));
  
    estimator->update();
    // Use the controller to send a command for every trajectory point
    controller->sendCommand(points, i, t);
    robot->current.print();

    wait(t, sec);
  }
  robot->stopAll();
}
#pragma endregion


#pragma region Lab5Examples
// Try setting estimator, controllers, and generators and execute trajectories one-by-one
void example5_1() {
  Driver driver = Driver(Pose(0, 0, 0));

  driver.setEstimator<DirectEstimator>();
  driver.setController<FeedforwardController>();
  driver.setTurnController<FeedforwardController>();

  driver.setGenerator<StraightGenerator>(20);
  driver.points = driver.generator->getPoints();
  driver.run();

  driver.setGenerator<TurnGenerator>(-90);
  driver.points = driver.generator->getPoints();
  driver.run();

  driver.setGenerator<StraightGenerator>(20);
  driver.points = driver.generator->getPoints();
  driver.run();

  driver.setGenerator<TurnGenerator>(90);
  driver.points = driver.generator->getPoints();
  driver.run();

  driver.printCurrent();
}


// Try executing multiple trajectories at a time and switch controller
void example5_2() {
  Driver driver = Driver(Pose(0, 0, 0));

  //driver.setEstimator<OdometryEstimator>();
  driver.setEstimator<OdometryIMUEstimator>();
  
  driver.setController<PIDController>();
  driver.setTurnController<PIDTurnController>();

  driver.straight(20);
  driver.turn(-90);
  driver.run();

  driver.setController<PurePursuitController>();
  
  driver.straight(20);
  driver.turn(90);
  driver.run();

  driver.printCurrent();
}


// Try executing a chain of relative trajectories
void example5_3() {
  Driver driver = Driver(Pose(0, 0, 0));

  driver.setEstimator<OdometryEstimator>();
  //driver.setEstimator<OdometryIMUEstimator>();
  
  driver.setController<PIDController>();
  driver.setTurnController<PIDTurnController>();

  driver.relative(20, -5, 0);
  driver.relative(20, 5, 0);
  driver.relative(20, -5, 0);
  driver.run();

  driver.printCurrent();
}


// Try driving around the field with absolute trajectories
void example5_4() {
  Driver driver = Driver(Pose(36, 0, 90));

  //driver.setEstimator<OdometryIMUEstimator>();
  driver.setEstimator<FusionEstimator>();

  //driver.setController<FeedforwardPIDController>();
  driver.setController<PurePursuitController>();
  
  driver.setTurnController<PIDTurnController>();

  driver.absolute(24, 28, 180);
  /*driver.absolute(-24, 28, 180);
  driver.absolute(-48, 48, 90);
  driver.absolute(-24, 64, 0);
  driver.absolute(24, 64, 0);
  driver.absolute(48, 48, -90);
  driver.run();

  driver.absolute(24, 28, 180);
  driver.absolute(-24, 28, 180);
  driver.run();

  //driver.robot->setPercent(10);
  driver.setController<FeedforwardPIDController>();
  driver.absolute(-32, 20, -90);
  driver.absolute(-36, 0, -90);
  driver.absolute(-32, -20, -90);
  driver.run();

  driver.setController<PurePursuitController>();
  driver.absolute(-24, -28, 0);
  driver.absolute(24, -28, 0);
  driver.absolute(48, -48, -90);
  driver.absolute(24, -64, 180);
  driver.absolute(-24, -64, 180);
  driver.absolute(-48, -48, 90);
  driver.run();

  driver.absolute(-24, -28, 0);
  driver.absolute(24, -28, 0);
  driver.run();

  driver.setController<FeedforwardPIDController>();
  driver.absolute(32, -20, 90);
  driver.absolute(36, 0, 90);
  driver.run();

  driver.absolute(36, 0, 90);
  driver.printPoints();
  driver.run(true);

  driver.setController<PurePursuitController>();

  // Print all planned trajectory points
  //driver.printPoints();
  // Print pose estimate while driving
  //driver.run(true);
  */
  driver.run();
}
#pragma endregion


#endif