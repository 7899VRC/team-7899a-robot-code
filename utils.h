#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <iomanip>

using namespace vex;


brain Brain;

// Degrees -> radians
inline float toRadians(float degrees) {
  return degrees * M_PI / 180.0;
}


// Radians -> degrees
inline float toDegrees(float radians) {
  return radians * 180.0 / M_PI;
}


// Fix angle so that -180 < angle <= 180
inline float normalizeAngle(float degrees) {
  while (degrees <= -180) {
    degrees += 360;
  }
  while (degrees > 180) {
    degrees -= 360;
  }
  return degrees;
}


class Pose {
  public:
    float x;
    float y;
    float deg;
    float rad;
    
    Pose() = default;

    Pose(float x, float y, float deg) { // Angle input is in degrees
      this->x = x;
      this->y = y;
      this->deg = deg; // Angle is internally stored in degrees
      this->rad = toRadians(deg); // Angle is internally converted to radians
    }
    
    void print() {
      std::cout << std::fixed << std::setprecision(2) << this->x << ", " << this->y << ", " << this->deg << std::endl;
    }

    void printBrain(float horizontal = 100, float vertical = 100) {
        Brain.Screen.printAt(horizontal, vertical, "(%.2f, %.2f, %.2f)", this->x, this->y, this->deg);
    }
};


class Command {
  public:
    float v; // Linear velocity
    float w; // Angular velocity

    Command() = default;

    Command(float v, float w) {
      this->v = v;
      this->w = w;
    }

    void print() {
      std::cout << std::fixed << std::setprecision(2) << this->v << ", " << this->w << std::endl;
    }
};


// absolute pose + relative pose -> absolute pose
inline Pose absoluteTransform(Pose start, Pose offset) {
  float x = start.x + (offset.x * cos(start.rad) - offset.y * sin(start.rad));
  float y = start.y + (offset.x * sin(start.rad) + offset.y * cos(start.rad));
  float th = normalizeAngle(start.deg + offset.deg);
  
  Pose end = Pose(x, y, th);
  return end;
}


// absolute pose + absolute pose -> relative pose
inline Pose relativeTransform(Pose start, Pose end) {
  float x = (end.x - start.x) * cos(start.rad) + (end.y - start.y) * sin(start.rad);
  float y = -(end.x - start.x) * sin(start.rad) + (end.y - start.y) * cos(start.rad);  
  float th = normalizeAngle(end.deg - start.deg);

  Pose offset = Pose(x, y, th);
  return offset;
}


// Calculate the distance between two poses
inline float calculateDistance(Pose start, Pose end) {
  return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
}


#endif