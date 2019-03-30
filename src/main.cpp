#include <Arduino.h>
#include <Enes100Simulation.h>
#include <TankSimulation.h>

#define ARENA_HEIGHT 2
#define ARENA_WIDTH 4

double getAngleToDest();
double getDistToDest();
void goAroundObstacle();

void updateLocation();

void stop();
void turn(double targetAngle);
void drive(int speed);

void setup() {
  TankSimulation.begin();
  while (!Enes100Simulation.begin()) {
    Enes100Simulation.println("Unable to connect to simulation");
  }

  Enes100Simulation.println("Starting Navigation");

  updateLocation();
}

void loop() {
  updateLocation();

  Enes100Simulation.print("Current X: ");
  Enes100Simulation.println(Enes100Simulation.location.x);
  Enes100Simulation.print("Current Y: ");
  Enes100Simulation.println(Enes100Simulation.location.y);

  if (Enes100Simulation.location.x < 1 && Enes100Simulation.location.y > 0.45) {
    turn(-PI / 2);
    drive(255);
    while (Enes100Simulation.location.y > 0.45) {
      updateLocation();
    }
    stop();
  } else if (Enes100Simulation.location.x < 3) {
    turn(0);
    drive(255);
    while (Enes100Simulation.readDistanceSensor(0) > 0.25 &&
           Enes100Simulation.readDistanceSensor(2) > 0.25 &&
           Enes100Simulation.location.x < 3) {
      updateLocation();
    }

    stop();
    if (Enes100Simulation.readDistanceSensor(0) <= 0.25 ||
        Enes100Simulation.readDistanceSensor(2) <= 0.25) {
      goAroundObstacle();
    }
  } else {
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

      if (getDistToDest() < 0.5) {
        delay(100);
      } else {
        delay(1000);
      }

      // stop motors
      stop();
    }
  }

  stop();
}

double getAngleToDest() {
  updateLocation();
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
  updateLocation();
  turn(PI / 4);
  drive(255);
  updateLocation();

  double currentX = Enes100Simulation.location.x;
  double targetX = currentX + 0.55;

  int offset = 0;

  while (currentX < targetX) {
    updateLocation();
    currentX = Enes100Simulation.location.x;
    if (Enes100Simulation.location.y < ARENA_WIDTH / 3) {
      if (Enes100Simulation.readDistanceSensor(0) >
              Enes100Simulation.readDistanceSensor(2) &&
          Enes100Simulation.readDistanceSensor(2) < 0.3) {
        offset += PI / 20;
        turn(PI / 4 + offset);
      } else if (Enes100Simulation.readDistanceSensor(0) <
                     Enes100Simulation.readDistanceSensor(2) &&
                 Enes100Simulation.readDistanceSensor(0) < 0.3) {
        offset -= PI / 20;
        turn(PI / 4 + offset);
      }
    }

    delay(100);
  }
  stop();
  turn(-PI / 3);
  drive(255);
  double currentY = Enes100Simulation.location.y;
  while (currentY > 0.4) {
    updateLocation();
    currentY = Enes100Simulation.location.y;
  }

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

    updateLocation();
  }
}

void drive(int speed) {
  TankSimulation.setLeftMotorPWM(speed);
  TankSimulation.setRightMotorPWM(speed);
}

void updateLocation() {
  while (!Enes100Simulation.updateLocation()) {
    Enes100Simulation.println("Unable to update location");
  }
}