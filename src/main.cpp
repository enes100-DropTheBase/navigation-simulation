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

float pingLeft();
float pingRight();

String status = "";

void setup() {
  TankSimulation.begin();
  while (!Enes100Simulation.begin()) {
    Enes100Simulation.println("Unable to connect to simulation");
  }

  Enes100Simulation.println("Starting Navigation");

  updateLocation();
}

void loop() {
  stop();
#ifdef SERIAL_DEBUG
  Serial.print("Right: ");
  Serial.println(Enes100Simulation.readDistanceSensor(2));
  Serial.print("Left: ");
  Serial.println(Enes100Simulation.readDistanceSensor(0));
#endif

  updateLocation();
  status = "";

  Enes100Simulation.print("Current X: ");
  Enes100Simulation.println(Enes100Simulation.location.x);
  Enes100Simulation.print("Current Y: ");
  Enes100Simulation.println(Enes100Simulation.location.y);

  if (Enes100Simulation.location.x < 1 && Enes100Simulation.location.y > 0.45) {
    // Go to the bottom corner
    status = "Going to bottom corner";
    Enes100Simulation.println(status.c_str());

    turn(-PI / 2);
    moveForward(255);
    while (Enes100Simulation.location.y > 0.45) {
      // TODO: make it slow down when getting close
      stop();
      updateLocation();
      moveForward(255);
      delay(100);
    }
    stop();
  } else if (Enes100Simulation.location.x < 3) {
    status = "Going across bottom";
    Enes100Simulation.println(status.c_str());
    turn(0);
    moveForward(255);
    Enes100Simulation.print("Right: ");
    Enes100Simulation.println(Enes100Simulation.readDistanceSensor(2));
    Enes100Simulation.print("Left: ");
    Enes100Simulation.println(Enes100Simulation.readDistanceSensor(0));
    while (Enes100Simulation.location.x < 1 ||
           ((pingLeft() > 50 || pingLeft() == 0) &&
            (pingRight() > 50 || pingRight() == 0) &&
            Enes100Simulation.location.x < 3)) {
      stop();

      Enes100Simulation.print("Right: ");
      Enes100Simulation.println(pingRight());
      Enes100Simulation.print("Left: ");
      Enes100Simulation.println(pingLeft());

      updateLocation();

      moveForward(255);

      delay(200);
      // TODO: periodically recheck angle and adjust if off course
    }

    stop();
    if (pingLeft() <= 50 || pingRight() <= 50) {
      status = "Going around obstacle";
      Enes100Simulation.println(status.c_str());
      goAroundObstacle();
    }
  } else if (Enes100Simulation.location.x < 4 &&
             Enes100Simulation.location.x >= 3 && getDistToDest() > 0.2) {
    status = "Going to destination";
    Enes100Simulation.println(status.c_str());
    turn(0);

    while (Enes100Simulation.location.x < Enes100Simulation.destination.x) {
      stop();
      updateLocation();
      moveForward(255);
      delay(200);
      stop();
      // TODO: periodically recheck angle and adjust if off course
    }
    Enes100Simulation.print("Dest y: ");
    Enes100Simulation.println(Enes100Simulation.destination.y);
    Enes100Simulation.print("OSV y: ");
    Enes100Simulation.println(Enes100Simulation.location.y);

    // TODO: go backwards if overshot
    if (Enes100Simulation.destination.y > Enes100Simulation.location.y) {
      Enes100Simulation.println("Destination Above");
      turn(PI / 2);
    } else {
      Enes100Simulation.println("Destination Below");
      turn(-PI / 2);
    }
    status = "Going to destination vertically";
    while (fabs(Enes100Simulation.destination.y -
                Enes100Simulation.location.y) > 0.2) {
      // move forward
      moveForward(255);

      if (fabs(Enes100Simulation.destination.y - Enes100Simulation.location.y) <
          0.5) {
        moveForward(200);
        delay(100);
      } else {
        delay(200);
      }

      // stop motors
      stop();
      updateLocation();
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
  status = "Avoiding Obstacle";
  Enes100Simulation.println("Avoiding Obstacle");
  stop();
  updateLocation();

  turn(PI / 4);
  updateLocation();

  double currentX = Enes100Simulation.location.x;
  double currentY = Enes100Simulation.location.y;

  double targetX = currentX + 0.75;
  double targetY = 0.7;

  // int offset = 0;

  while (currentY < targetY) {
    stop();
    updateLocation();
    currentY = Enes100Simulation.location.y;
    moveForward(255);
    delay(100);
  }

  turn(0);

  updateLocation();

  currentX = Enes100Simulation.location.x;

  while (currentX < targetX) {
    stop();
    updateLocation();
    currentX = Enes100Simulation.location.x;
    moveForward(255);
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
  while (angleDifference > 0.05) {
    int speed = 200;
    if (angleDifference < 0.3) {
      speed = 150;
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
  while (bool loc = !Enes100Simulation.updateLocation() ||
                    Enes100Simulation.location.theta > 10 ||
                    Enes100Simulation.location.x > 10 ||
                    Enes100Simulation.location.y > 10 ||
                    Enes100Simulation.location.x < 0.02 ||
                    Enes100Simulation.location.y < -0.02 ||
                    Enes100Simulation.location.theta < -10) {
    if (!loc) {
      Enes100Simulation.println("Invalid location");
    } else {
#ifdef DEBUG_UPDATE_LOCATION
      Enes100Simulation.println("Unable to update location");
#endif
    }
    delay(100);
  }
  Enes100Simulation.print(status.c_str());
  Enes100Simulation.print(": ");
  Enes100Simulation.print("OSV is at (");
  Enes100Simulation.print(Enes100Simulation.location.x);
  Enes100Simulation.print(", ");
  Enes100Simulation.print(Enes100Simulation.location.y);
  Enes100Simulation.print(", ");
  Enes100Simulation.print(Enes100Simulation.location.theta);
  Enes100Simulation.println(")");
}

float pingLeft() { return Enes100Simulation.readDistanceSensor(0) * 100; }

float pingRight() { return Enes100Simulation.readDistanceSensor(2) * 100; }