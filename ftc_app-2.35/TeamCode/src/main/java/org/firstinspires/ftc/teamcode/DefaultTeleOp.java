package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class DefaultTeleOp extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		double DEFAULT_SPEED_COEFFICIENT = 1;
		double SLOW_SPEED_COEFFICIENT = 0.25;
		double DEAD_ZONE = 0.1;

		double rightMotorSpeed;
		double leftMotorSpeed;

		initializeRobot();

		waitForStart();

		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

			if (gamepad1.right_bumper)
			{
				rightMotorSpeed = rightMotorSpeed * SLOW_SPEED_COEFFICIENT;
				leftMotorSpeed = leftMotorSpeed * SLOW_SPEED_COEFFICIENT;
			}
			else
			{
				rightMotorSpeed = rightMotorSpeed * DEFAULT_SPEED_COEFFICIENT;
				leftMotorSpeed = leftMotorSpeed * DEFAULT_SPEED_COEFFICIENT;
			}

			if (gamepad2.right_trigger > 0)
			{
				ballDeployer.setPosition(0.75);
			}
			else
			{
				ballDeployer.setPosition(1);
			}


			if (gamepad2.left_bumper)
			{
				ballCollector.setPower(1);
			}
			else if (gamepad2.right_bumper)
			{
				ballCollector.setPower(-1);
			}
			else
			{
				ballCollector.setPower(0);
			}

			if (gamepad2.left_trigger > 0)
			{
				ballLauncher.setPower(0.65);
			}
			else
			{
				ballLauncher.setPower(0);
			}

			//TODO Fix this mess. The controls are literally flipped AND reversed
			rightMotor.setPower(-leftMotorSpeed);
			leftMotor.setPower(-rightMotorSpeed);
		}
	}
}
