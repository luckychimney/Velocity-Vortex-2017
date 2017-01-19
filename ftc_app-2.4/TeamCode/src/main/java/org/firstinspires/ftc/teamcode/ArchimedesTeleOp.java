package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class ArchimedesTeleOp extends Archimedes
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		double DEFAULT_SPEED_COEFFICIENT = 1;
		double SLOW_SPEED_COEFFICIENT = 0.25;
		double DEAD_ZONE = 0.1;

		double rightMotorSpeed;
		double leftMotorSpeed;

		boolean isDriveControlsReversed = false;

		initializeArchimedes();
		waitForStart();

		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		while (opModeIsActive())
		{
			/*
			* Driver
			 */

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

			if (gamepad1.start)
			{
				isDriveControlsReversed = !isDriveControlsReversed;
				while (gamepad1.start)
				{
					idle();
				}
			}
			/*
			* Gunner
			 */

			if (gamepad2.right_trigger > 0)
			{
				liftBallDeployer();
			}
			else
			{
				dropBallDeployer();
			}

			if (gamepad2.left_trigger > 0)
			{
				ballLauncher.setPower(0.75);
			}
			else
			{
				ballLauncher.setPower(0);
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

			if (gamepad2.dpad_up)
			{
				liftCapBallGrabber();
			}

			if (gamepad2.dpad_left || gamepad2.dpad_right)
			{
				clampCapBallGrabber();
			}

			if (gamepad2.dpad_down)
			{
				dropCapBallGrabber();
			}

			if (gamepad2.b)
			{
				turnButtonPusherRight();
			}

			if (gamepad2.x)
			{
				turnButtonPusherLeft();
			}

			if (gamepad2.a || gamepad2.y)
			{
				setButtonPusherToNeutral();
			}

			liftMotor1.setPower(-gamepad2.right_stick_y);
			liftMotor2.setPower(liftMotor1.getPower());

			//FIXME The controls are literally flipped AND reversed

			if (isDriveControlsReversed)
			{
				rightMotor.setPower(rightMotorSpeed);
				leftMotor.setPower(leftMotorSpeed);
			}
			else
			{
				rightMotor.setPower(-leftMotorSpeed);
				leftMotor.setPower(-rightMotorSpeed);
			}
		}
	}
}
