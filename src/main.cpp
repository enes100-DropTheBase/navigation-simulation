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
void moveForward(int speed);

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
    // Go to the bottom corner
    turn(-PI / 2);
    moveForward(255);
    while (Enes100Simulation.location.y > 0.45) {
      // TODO: make it slow down when getting close
      updateLocation();
    }
    stop();
  } else if (Enes100Simulation.location.x < 3) {
    // Go across the bottom
    turn(0);
    moveForward(255);
    while (  // rightSonar.ping_cm() > 0.25 && leftSonar.ping_cm() > 0.25 &&
        Enes100Simulation.location.x < 3) {
      updateLocation();
      // TODO: periodically recheck angle and adjust if off course
    }

    stop();
    // if (rightSonar.ping_cm() <= 0.25 || leftSonar.ping_cm() <= 0.25) {
    //   goAroundObstacle();
    // }
  } else {
    double targetAngle = getAngleToDest();
    if (getDistToDest() > 0.1) {
      // Go to the destination
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
      moveForward(255);

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
  updateLocation();
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
  moveForward(255);
  updateLocation();

  double currentX = Enes100Simulation.location.x;
  double targetX = currentX + 0.55;

  // int offset = 0;

  while (currentX < targetX) {
    updateLocation();
    currentX = Enes100Simulation.location.x;
    // if (Enes100Simulation.location.y < ARENA_WIDTH / 3) {
    //   if (rightSonar.ping_cm() > leftSonar.ping_cm() &&
    //       leftSonar.ping_cm() < 0.2) {
    //     offset += PI / 20;
    //     Enes100Simulation.println("Compensating left");
    //     turn(PI / 4 + offset);
    //   } else if (rightSonar.ping_cm() < leftSonar.ping_cm() &&
    //              rightSonar.ping_cm() < 0.2) {
    //     Enes100Simulation.println("Compensating right");
    //     offset -= PI / 20;
    //     turn(PI / 4 + offset);
    //   }
    // }

    delay(100);
  }
  stop();
  updateLocation();

  Enes100Simulation.print("Current X: ");
  Enes100Simulation.println(Enes100Simulation.location.x);
  Enes100Simulation.print("Current Y: ");
  Enes100Simulation.println(Enes100Simulation.location.y);

  if (Enes100Simulation.location.x < 2.5) {
    turn(-PI / 2.7);
    moveForward(255);
    double currentY = Enes100Simulation.location.y;
    while (currentY > 0.4) {
      updateLocation();
      currentY = Enes100Simulation.location.y;
    }
  }

  stop();
}

void stop() {
  TankSimulation.setLeftMotorPWM(0);
  TankSimulation.setRightMotorPWM(0);
}

void turnRight(int speed) {
  TankSimulation.setLeftMotorPWM(speed);
  TankSimulation.setRightMotorPWM(-speed);
}

void turnLeft(int speed) {
  TankSimulation.setLeftMotorPWM(-speed);
  TankSimulation.setRightMotorPWM(speed);
}

void moveForward(int speed) {
  TankSimulation.setLeftMotorPWM(speed);
  TankSimulation.setRightMotorPWM(speed);
}

void turn(double targetAngle) {
  updateLocation();
  // Enes100Simulation.print("Difference");
  // Enes100Simulation.println(fabs(Enes100Simulation.location.theta -
  // targetAngle));
  // TODO: this is too reliant on the vision system
  double angleDifference = fabs(Enes100Simulation.location.theta - targetAngle);
  while (angleDifference > 0.09) {
    int speed = 200;
    if (angleDifference < 0.3) {
      speed = 125;
    }
    if (Enes100Simulation.location.theta - targetAngle > 0) {
      turnRight(speed);
    } else {
      turnLeft(speed);
    }
    delay(100);
    stop();

    updateLocation();

    angleDifference = fabs(Enes100Simulation.location.theta - targetAngle);
  }
}

void updateLocation() {
  while (!Enes100Simulation.updateLocation()) {
    Enes100Simulation.println("Unable to update location");
  }
}