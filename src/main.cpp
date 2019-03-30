#include <Arduino.h>
#include <Enes100Simulation.h>
#include <TankSimulation.h>

#define ARENA_HEIGHT 2
#define ARENA_WIDTH 4

double getAngleToDest();
double getDistToDest();
void goAroundObstacle();

void stop();
void turn(double targetAngle);
void drive(int speed);

void setup() {
  TankSimulation.begin();
  while (!Enes100Simulation.begin()) {
    Enes100Simulation.println("Unable to connect to simulation");
  }

  Enes100Simulation.println("Starting Navigation");

  while (!Enes100Simulation.updateLocation()) {
    Enes100Simulation.println("Unable to update Location");
  }
}

void loop() {
  if (Enes100Simulation.readDistanceSensor(1) < 0.1) {
    goAroundObstacle();
  }

  double targetAngle = getAngleToDest();

  if (getDistToDest() > 0.1) {
    if (Enes100Simulation.location.x > Enes100Simulation.destination.x) {
      if (targetAngle < 0) {
        targetAngle += PI;
      } else {
        targetAngle -= PI;
      }
    }

    Enes100Simulation.print("Distance to destination: ");
    Enes100Simulation.println(getDistToDest());

    Enes100Simulation.print("Target Angle: ");
    Enes100Simulation.println(targetAngle * 180 / PI);

    // turn to face destination
    turn(targetAngle);

    // move forward
    drive(255);

    Enes100Simulation.println(Enes100Simulation.readDistanceSensor(1));

    if (Enes100Simulation.readDistanceSensor(1) > 0.1) {
      if (getDistToDest() < 0.5) {
        delay(100);
      } else {
        delay(1000);
      }
    }

    // stop motors
    stop();
  }
}

double getAngleToDest() {
  // TODO: add checks to make sure location can be updated
  Enes100Simulation.updateLocation();
  double deltaX =
      Enes100Simulation.location.x - Enes100Simulation.destination.x;
  double deltaY =
      Enes100Simulation.location.y - Enes100Simulation.destination.y;

  // Enes100Simulation.println(deltaX);
  // Enes100Simulation.println(deltaY);

  double angle = atan(deltaY / deltaX);

  // Enes100Simulation.println(angle);

  return angle;
}

double getDistToDest() {
  // TODO: add checks to make sure location can be updated
  Enes100Simulation.updateLocation();
  double deltaX =
      Enes100Simulation.location.x - Enes100Simulation.destination.x;
  double deltaY =
      Enes100Simulation.location.y - Enes100Simulation.destination.y;

  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

void goAroundObstacle() {
  Enes100Simulation.println("Avoiding Obstacle");

  Enes100Simulation.println(Enes100Simulation.location.y);

  // back up a bit
  drive(-255);
  delay(500);

  if (Enes100Simulation.location.y > ARENA_HEIGHT / 2) {
    // go down and around
    turn(-PI / 2 + PI / 8);
  } else {
    // go up and around
    turn(PI / 2 - PI / 8);
  }

  // move forward
  drive(255);
  delay(3000);
  stop();
}

void stop() {
  TankSimulation.setLeftMotorPWM(0);
  TankSimulation.setRightMotorPWM(0);
}
void turn(double targetAngle) {
  // TODO: this is too reliant on the vision system
  while (abs(Enes100Simulation.location.theta - targetAngle) > 0.05) {
    if (Enes100Simulation.location.theta - targetAngle > 0) {
      TankSimulation.setLeftMotorPWM(255);
      TankSimulation.setRightMotorPWM(-255);
    } else {
      TankSimulation.setLeftMotorPWM(-255);
      TankSimulation.setRightMotorPWM(255);
    }

    while (!Enes100Simulation.updateLocation()) {
      Enes100Simulation.println("Unable to update location");
    }
  }
}
void drive(int speed) {
  TankSimulation.setLeftMotorPWM(speed);
  TankSimulation.setRightMotorPWM(speed);
}