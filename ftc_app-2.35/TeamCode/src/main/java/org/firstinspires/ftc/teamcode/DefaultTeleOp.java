package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class DefaultTeleOp extends LinearOpMode
{
	private Robot robot = new Robot();

	@Override
	public void runOpMode() throws InterruptedException
	{
		double DEFAULT_SPEED_COEFFICIENT = .75;
		double SLOW_SPEED_COEFFICIENT = 0.25;
		double DEAD_ZONE = 0.1;

		double rightMotorSpeed;
		double leftMotorSpeed;

		robot.init(hardwareMap);

		waitForStart();

		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		while (opModeIsActive())
		{
			if (Math.abs(gamepad1.right_stick_y) > DEAD_ZONE)
			{
				rightMotorSpeed = gamepad1.right_stick_y;
			}
			else
			{
				rightMotorSpeed = 0;
			}

			if (Math.abs(gamepad1.left_stick_y) > DEAD_ZONE)
			{
				leftMotorSpeed = gamepad1.left_stick_y;
			}
			else
			{
				leftMotorSpeed = 0;
			}

			if (gamepad1.left_bumper)
			{
				rightMotorSpeed = rightMotorSpeed * SLOW_SPEED_COEFFICIENT;
				leftMotorSpeed = leftMotorSpeed * SLOW_SPEED_COEFFICIENT;
			}
			else
			{
				rightMotorSpeed = rightMotorSpeed * DEFAULT_SPEED_COEFFICIENT;
				leftMotorSpeed = leftMotorSpeed * DEFAULT_SPEED_COEFFICIENT;
			}

			// TODO add the other functions of the robot to tele op

			robot.rightMotor.setPower(-rightMotorSpeed);
			robot.leftMotor.setPower(-leftMotorSpeed);
		}
	}
}
