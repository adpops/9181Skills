#include "main.h"




/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::Motor leftBDrive_mtr(2, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(3, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

	pros::Motor flyWheel_mtr(5, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor indexer(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor capScorer(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

	pros::ADIUltrasonic leftSensor('A', 'B');
	pros::ADIUltrasonic rightSensor('C', 'D');
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::lcd::set_text(4, "MOTOR SPEED=" + std::to_string(leftFDrive_mtr.get_position()));

	int speed = 0;

	while (true) {

		leftBDrive_mtr.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_LEFT_X));
		leftFDrive_mtr.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_LEFT_X));
		rightFDrive_mtr.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_LEFT_X));
		rightBDrive_mtr.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_LEFT_X));

		flyWheel_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		capScorer.move(master.get_analog(ANALOG_RIGHT_Y));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
      for(int i = 1; i <= 127; i++)
			{
				speed++;
			}
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			speed = 0;
		}

		flyWheel_mtr.move(speed);

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			ballIntake_mtr.move(100);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			ballIntake_mtr.move(-100);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			indexer.move(100);
		}
		else
		{
			indexer.move(0);
			ballIntake_mtr.move(0);
		}


    pros::delay(20);
	}
}
