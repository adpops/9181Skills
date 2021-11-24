#include <math.h>
#include "main.h"

static pros::Controller * controllerMain = new pros::Controller(CONTROLLER_MAIN);

static pros::Motor * backLeftDrive = new pros::Motor(9, GEARSET_200, REV, ENCODER_DEGREES);
static pros::Motor * frontLeftDrive = new pros::Motor(12, GEARSET_200, REV, ENCODER_DEGREES);
static pros::Motor * intakeMotor = new pros::Motor(13, GEARSET_200, REV, ENCODER_DEGREES);
static pros::Motor * frontLauncherMotor = new pros::Motor(14, GEARSET_200, REV, ENCODER_DEGREES);
static pros::Motor * backLauncherMotor = new pros::Motor(15, GEARSET_200, FWD, ENCODER_DEGREES);

static pros::Motor * liftMotor = new pros::Motor(18, GEARSET_200, REV, ENCODER_DEGREES);
static pros::Motor * frontRightDrive = new pros::Motor(19, GEARSET_200, FWD, ENCODER_DEGREES);
static pros::Motor * backRightDrive = new pros::Motor(20, GEARSET_200, FWD, ENCODER_DEGREES);

int autoCounter = 0;

/**
* Runs the user autonomous code-> This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode-> Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes->
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped-> Re-enabling the robot will restart the task, not re-start it
* from where it left off->
*/

static void driveMotors(int powerLeft, int powerRight)
{
 frontLeftDrive->move(powerLeft);
 backLeftDrive->move(powerLeft);
 backRightDrive->move(powerRight);
 frontRightDrive->move(powerRight);
}

static void resetPos()
{
  backLeftDrive->tare_position();
  backRightDrive->tare_position();
  frontRightDrive->tare_position();
  frontLeftDrive->tare_position();
}

static void resetSensor(int target)
{
 frontLeftDrive->set_zero_position(target);
 backRightDrive->set_zero_position(target);
 backLeftDrive->set_zero_position(target);
 frontRightDrive->set_zero_position(target);
}

static void driveBrakeHold()
{
 frontLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 backLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 frontRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 backRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

static void resetBrake()
{
 frontLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 backLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 frontRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 backRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

static void movePID(int power)
{

  double kp = 1; //0->6485

  int main = 0;
  int secondary = 0;
  int error = 1;
  int powerLeft, powerRight;

  int maxPower = power;
  int MIN_POWER = maxPower - 1;
  //int leftPos = (abs(lfdMotor->get_position()) + abs(lbdMotor->get_Position())/2;
  main = (abs(frontRightDrive->get_position()) >= abs(frontLeftDrive->get_position())) ? abs(frontRightDrive->get_position()) : abs(frontLeftDrive->get_position());
  secondary = (abs(frontRightDrive->get_position()) >= abs(frontLeftDrive->get_position())) ? abs(frontLeftDrive->get_position()) : abs(frontRightDrive->get_position());

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

  if(frontRightDrive->get_position() > frontLeftDrive->get_position())
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

 resetPos();
 degrees*=3.2;

 frontLeftDrive->move_relative(-degrees, -power);
 backLeftDrive->move_relative(-degrees, -power);
 backRightDrive->move_relative(degrees, power);
 frontRightDrive->move_relative(degrees, power);

 while (!((frontLeftDrive->get_position() < (degrees + 5)) && (frontLeftDrive->get_position() > (degrees - 5)))
        && !((frontRightDrive->get_position() < (-degrees + 5)) && (frontRightDrive->get_position() > (-degrees - 5))))
 {
  pros::delay(2);
 }
 driveBrakeHold();
}

static void flyCoast()
{

 frontLauncherMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 backLauncherMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void moveDrive(double targetDistance, int maxPower, int flywheelP, int indexerP, int intakeP)
{
 int currentDistance = 0;
 double error = 20;

 int targ = (targetDistance/12.566) * 360;

 int leftF, rightF;

 resetPos();


  while(error != 0)
  {
     leftF = abs(frontLeftDrive->get_position());
     rightF = abs(frontRightDrive->get_position());

     currentDistance = (leftF + rightF)/2;

     error = targ - currentDistance;

     frontLauncherMotor->move(flywheelP);
     backLauncherMotor->move(flywheelP);
     intakeMotor->move(intakeP);
     //indexer->move(indexerP);

     driveMotors(maxPower, maxPower);
     //movePID(maxPower);

     if(error < 15 && error > -15)
     {
       error = 0;
       driveMotors(0, 0);
       driveBrakeHold();
       flyCoast();
     }
   }
   resetBrake();

   intakeMotor->move(0);
  }

void miscell(int powerIntake, int powerIndexer, int time)
{

 intakeMotor->move(powerIntake);
 //indexer->move(powerIndexer);

 pros::delay(time);
}

void flywheelAccel(int maxSpeed)
{
  int speed = 0;


  for(int i = 1; i <= maxSpeed; i++)
  {
    speed++;
  }

  frontLauncherMotor->move(speed);
}
/* static void sonarReset(int power)
{
 pros::Motor backLeftDrive(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor backRightDrive(12, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::Motor frontLeftDrive(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
 pros::Motor frontRightDrive(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
 pros::ADIUltrasonic leftSensor('A', 'B');
 pros::ADIUltrasonic rightSensor('C', 'D');
 if(leftSensor->get_value() > rightSensor->get_value())
 {
   while(leftSensor->get_value() > rightSensor->get_value())
   {
       turnPID(-power);
   }
   driveBrakeHold();
 }
 else if(leftSensor->get_value() < rightSensor->get_value())
 {
   while(leftSensor->get_value() < rightSensor->get_value())
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

void blueAuto()
{
  flyCoast();

  //gets back cap

  flywheelAccel(127);

  moveDrive(35, 87, 127, 0, 0);

  moveDrive(10, 90, 127, 0, 0);
  miscell(70, 0, 350);
  stop(150);
  moveDrive(10, -60, 127, 0, 0);

  driveTurn(84, 80);
  flywheelAccel(102);
  stop(1000);
  miscell(0, 100, 500);

  flywheelAccel(93);
  stop(1500);
  miscell(65, 120, 1000);
  stop(200);

  driveTurn(-135, 80);
  moveDrive(25, 85, 127, 0, -120);
  miscell(-80, 0, 350);
  moveDrive(12, -80, 127, 0, 0);
  driveTurn(-40, 80);

  moveDrive(42, -127, 127, 0, 0);
}

void redAuto()
{
  flyCoast();

  //gets back cap
  flywheelAccel(127);

  moveDrive(32, 87, 127, 0, 0);

  moveDrive(10, 90, 127, 0, 0);
  miscell(70, 0, 350);
  stop(150);
  moveDrive(10, -60, 127, 0, 0);

  driveTurn(-82, 80);
  flywheelAccel(107);
  stop(1000);
  miscell(0, 100, 500);

  flywheelAccel(93);
  stop(1500);
  miscell(65, 120, 1000);
  stop(200);

  driveTurn(135, 80);
  moveDrive(25, 85, 127, 0, -120);
  miscell(-80, 0, 350);
  moveDrive(12, -80, 127, 0, 0);
  driveTurn(40, 80);

  moveDrive(49, -127, 127, 0, 0);
}

void autonomous()
{
  autoCounter = (autoCounter < -1) ? -1 : autoCounter;
  autoCounter = (autoCounter > 1) ? 1 : autoCounter;

  switch(autoCounter)
  {
    case -1:
      redAuto();
      break;
    case 1:
      blueAuto();
      break;
  }
}
