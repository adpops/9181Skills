#include <math.h>
#include "main.h"

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/

static void driveMotors(int powerLeft, int powerRight)
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 leftFDrive_mtr.move(powerLeft);
 leftBDrive_mtr.move(powerLeft);
 rightBDrive_mtr.move(powerRight);
 rightFDrive_mtr.move(powerRight);
}

static void resetPos()
{
  pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
  pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
  pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
  pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

  leftBDrive_mtr.tare_position();
  rightBDrive_mtr.tare_position();
  rightFDrive_mtr.tare_position();
  leftFDrive_mtr.tare_position();
}

static void resetSensor(int target)
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);


 leftFDrive_mtr.set_zero_position(target);
 rightBDrive_mtr.set_zero_position(target);
 leftBDrive_mtr.set_zero_position(target);
 rightFDrive_mtr.set_zero_position(target);
}

static void driveBrakeHold()
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 leftFDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 leftBDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 rightFDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 rightBDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

static void resetBrake()
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 leftFDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 leftBDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 rightFDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 rightBDrive_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

static void movePID(int power)
{
  pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
  pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
  pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
  pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

  double kp = 0.6485;

  int main = 0;
  int secondary = 0;
  int error = 1;
  int powerLeft, powerRight;

  int maxPower = power;
  int MIN_POWER = maxPower - 1;
  //int leftPos = (abs(lfdMotor.get_position()) + abs(lbdMotor.get_Position())/2;
  main = (abs(rightFDrive_mtr.get_position()) >= abs(leftFDrive_mtr.get_position())) ? abs(rightFDrive_mtr.get_position()) : abs(leftFDrive_mtr.get_position());
  secondary = (abs(rightFDrive_mtr.get_position()) >= abs(leftFDrive_mtr.get_position())) ? abs(leftFDrive_mtr.get_position()) : abs(rightFDrive_mtr.get_position());

  error = main - secondary;

  if(main > secondary)
  {
    power = power - error * kp;
  }

  if(power > 0)
  {
   if(power < MIN_POWER)
   {
     power = MIN_POWER;
   }
  }
  else if(power < 0)
  {
   if(power > -MIN_POWER)
   {
     power = -MIN_POWER;
   }
  }

  if(rightFDrive_mtr.get_position() > leftFDrive_mtr.get_position())
  {
   powerLeft = maxPower;
   powerRight = power;
  }
  else
  {
   powerLeft = power;
   powerRight = maxPower;
  }

  driveMotors(powerLeft, powerRight);
}

static void driveTurn(int degrees, int power)
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 resetPos();
 degrees*=3.107;

 leftFDrive_mtr.move_relative(degrees, power);
 leftBDrive_mtr.move_relative(degrees, power);
 rightBDrive_mtr.move_relative(-degrees, -power);
 rightFDrive_mtr.move_relative(-degrees, -power);

 while (!((leftFDrive_mtr.get_position() < (degrees + 5)) && (leftFDrive_mtr.get_position() > (degrees - 5)))
        && !((rightFDrive_mtr.get_position() < (-degrees + 5)) && (rightFDrive_mtr.get_position() > (-degrees - 5))))
 {
  pros::delay(2);
 }
 driveBrakeHold();
}

static void flyCoast()
{
 pros::Motor flyWheel_mtr(10, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

 flyWheel_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void moveDrive(double targetDistance, int maxPower, int flyWheelP, int indexerP, int intakeP)
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor flyWheel_mtr(10, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

 pros::Motor ballIntake_mtr(3, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor indexer(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 int currentDistance = 0;
 double error = 20;

 int targ = (targetDistance/12.566) * 360;

 int leftF, rightF;

 resetPos();


while(error != 0)
{
   leftF = abs(leftFDrive_mtr.get_position());
   rightF = abs(rightFDrive_mtr.get_position());

   currentDistance = (leftF + rightF)/2;

   error = targ - currentDistance;

   ballIntake_mtr.move(intakeP);
   indexer.move(indexerP);

   flyWheel_mtr.move(flyWheelP);

   movePID(maxPower);

   if(error < 15 && error > -15)
   {
     error = 0;
     driveMotors(0, 0);
     driveBrakeHold();
     flyCoast();
   }
 }
 resetBrake();

 ballIntake_mtr.move(0);
}

void miscell(int powerIntake, int powerIndexer, int time)
{
 pros::Motor intake(3, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor indexer(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

 intake.move(powerIntake);
 indexer.move(powerIndexer);

 pros::delay(time);
}

void flywheelAccel(int maxSpeed)
{
  int speed = 0;

  pros::Motor flyWheel(10, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

  for(int i = 1; i <= maxSpeed; i++)
  {
    speed++;
  }

  flyWheel.move(speed);
}
/* static void sonarReset(int power)
{
 pros::Motor leftBDrive_mtr(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightBDrive_mtr(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor rightFDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::ADIUltrasonic leftSensor('A', 'B');
 pros::ADIUltrasonic rightSensor('C', 'D');
 if(leftSensor.get_value() > rightSensor.get_value())
 {
   while(leftSensor.get_value() > rightSensor.get_value())
   {
       turnPID(-power);
   }
   driveBrakeHold();
 }
 else if(leftSensor.get_value() < rightSensor.get_value())
 {
   while(leftSensor.get_value() < rightSensor.get_value())
   {
       turnPID(power);
   }
   driveBrakeHold();
 }
}*/

void stop(int x)
{
  pros::delay(x);
}

void skillsAuton()
{
//moveDrive(distance, power, flywheel, indexer, intake, tipper);
//miscell(tipper, intake, indexer, time);

  flyCoast();

  //gets back cap

  flywheelAccel(127);

  moveDrive(30, 87, 127, 0, 0);

  moveDrive(10, 90, 127, 0, -40);

  miscell(70, 0, 700);

  moveDrive(40, -87, 127, 0, 0);

  driveMotors(-57, -57);

  stop(500);

  moveDrive(7, 67, 127, 0, 0);

  stop(150);

  driveTurn(-90, -60);

  stop(150);

  moveDrive(40, 97, 127, 0, 0);

  stop(150);

  miscell(0, 80, 1000);

  moveDrive(25, 87, 127, 0, 0);

  stop(150);

  miscell(75, 80, 1000);

  moveDrive(10, 87, 127, 0, 0);

  driveTurn(-15, -67);

  moveDrive(12, 67, 127, 0, 0);

  moveDrive(15, -87, 127, 0, 0);

  driveTurn(15, 67);

  stop(150);

  moveDrive(28, -87, 127, 0, 0);

  //gets front cap

  driveTurn(90, 60);

  stop(150);

  driveMotors(-87, -87);

  pros::delay(700);

  stop(150);

  moveDrive(44, 97, 127, 0, -50);

  miscell(60, 0, 750);

  driveTurn(-90, -60);

  stop(150);

  miscell(70, 80, 1000);

  driveTurn(-5, -67);

  moveDrive(46, 87, 127, 0, 0);
}

void blueAuton()
{

}

void redAuton()
{

}

void autonomous()
{
  /*
  if(auton == 0)
  {
  }
  else if(auton == 1)
  {
  }
  else if(auton == 2)
  {
  }*/
  //skillsAuton();

  moveDrive(30, 90, 0, 0, 0);
}
