#include "main.h"

/**
 * Runs the operator control code-> This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode->
 *
 * If no competition control is connected, this function will run immediately
 * following initialize()->
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped-> Re-enabling the robot will restart the
 * task, not resume it from where it left off->
 */

void opcontrol() {

	int speed = 0;

	static pros::Controller * controllerMain = new pros::Controller(CONTROLLER_MAIN);

	static pros::Motor * backLeftDrive = new pros::Motor(9, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * frontLeftDrive = new pros::Motor(12, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * intakeMotor = new pros::Motor(13, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * frontLauncherMotor = new pros::Motor(14, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * backLauncherMotor = new pros::Motor(15, GEARSET_200, FWD, ENCODER_DEGREES);

	static pros::Motor * indexer = new pros::Motor(6, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * liftMotor = new pros::Motor(18, GEARSET_200, REV, ENCODER_DEGREES);
	static pros::Motor * frontRightDrive = new pros::Motor(19, GEARSET_200, FWD, ENCODER_DEGREES);
	static pros::Motor * backRightDrive = new pros::Motor(20, GEARSET_200, FWD, ENCODER_DEGREES);

	backLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	frontLeftDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	frontRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backRightDrive->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);



	while (true) {

		backLeftDrive->move(controllerMain->get_analog(ANALOG_LEFT_Y) - controllerMain->get_analog(ANALOG_LEFT_X)*3/4);
		frontLeftDrive->move(controllerMain->get_analog(ANALOG_LEFT_Y) - controllerMain->get_analog(ANALOG_LEFT_X)*3/4);
		frontRightDrive->move(controllerMain->get_analog(ANALOG_LEFT_Y) + controllerMain->get_analog(ANALOG_LEFT_X)*3/4);
		backRightDrive->move(controllerMain->get_analog(ANALOG_LEFT_Y) + controllerMain->get_analog(ANALOG_LEFT_X)*3/4);

		frontLauncherMotor->set_brake_mode(BRAKE_COAST);
		backLauncherMotor->set_brake_mode(BRAKE_COAST);

		liftMotor->move(controllerMain->get_analog(ANALOG_RIGHT_Y));

		if(controllerMain->get_digital(BUTTON_L1))
		{
      for(int i = 1; i <= 127; i++)
			{
				speed++;
			}
		}
		else if(controllerMain->get_digital(BUTTON_L2))
		{
			speed = 0;
		}

		frontLauncherMotor->move(speed);
		backLauncherMotor->move(speed);

    if(controllerMain->get_digital(BUTTON_R1))
		{
			intakeMotor->move(100);
		}
		else if(controllerMain->get_digital(BUTTON_R2))
		{
			intakeMotor->move(-100);
		}
		else if(controllerMain->get_digital(BUTTON_A))
		{
			indexer->move(100);
		}
		else if(controllerMain->get_digital(BUTTON_B))
		{
			indexer->move(-100);
		}
		else
		{
			indexer->move(0);
			intakeMotor->move(0);
		}

		if(controllerMain->get_digital(BUTTON_X))
		{
			autonomous();
		}
    pros::delay(20);
	}
}
