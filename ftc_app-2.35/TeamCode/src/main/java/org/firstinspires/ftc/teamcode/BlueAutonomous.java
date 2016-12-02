package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Blue Autonomous")
public class BlueAutonomous extends LinearOpMode
{
	private Robot robot = new Robot();

	private ElapsedTime elapsedTime = new ElapsedTime();

	@Override
	public void runOpMode() throws InterruptedException
	{
		robot.init(hardwareMap);

		telemetry.addData(">", "Robot ready!");
		telemetry.update();

		waitForStart();

		telemetry.addData(">", "Robot running...");
		telemetry.update();

		robot.ballLauncher.setPower(0.7);
		sleep(1500);
		robot.ballDeployer.setPosition(.75);
		sleep(1000);
		robot.ballDeployer.setPosition(1);
		sleep(1500);
		robot.ballDeployer.setPosition(.75);
		sleep(1000);
		robot.ballDeployer.setPosition(1);
		sleep(1000);
		robot.ballLauncher.setPower(0);

		drive(1, 1000);
		turn(.75, 175);
		timeDrive(-1, 1000);
	}

	private void drive(double power, int distance)
	{
		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			double encoderUnitsToDrive = robot.ENCODER_UNITS_PER_MILLIMETER * distance;

			robot.gyroSensor.resetZAxisIntegrator();

			if (distance < 0)
			{
				power = power * -1;
			}

			while ((Math.abs(robot.getEncoderWheel().getCurrentPosition()) < Math.abs(encoderUnitsToDrive) && opModeIsActive()))
			{
				leftAdjustedPower = Range.clip(power - getPowerAdjustment(), -1, 1);
				rightAdjustedPower = Range.clip(power + getPowerAdjustment(), -1, 1);
				robot.leftMotor.setPower(leftAdjustedPower);
				robot.rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);

				telemetry.addData("Angle", getAbsGyroHeading());
				telemetry.addData("Adjusted Power", getPowerAdjustment());
				telemetry.addData("Encoder", robot.getEncoderWheel().getCurrentPosition());
				telemetry.update();
			}
		}

		stopDriveMotors();
	}

	private void timeDrive(double power, int time)
	{
		if (opModeIsActive())
		{
			robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			robot.leftMotor.setPower(power);
			robot.rightMotor.setPower(power);

			double startTime = elapsedTime.milliseconds();

			while (startTime + time > elapsedTime.milliseconds() && opModeIsActive())
			{
				sleep(50);
				idle();
			}
		}

		stopDriveMotors();
	}

	private void turn(double power, int angle)
	{
		if (opModeIsActive())
		{
			robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			robot.gyroSensor.resetZAxisIntegrator();

			while (!isHeadingReached(angle) && opModeIsActive())
			{
				robot.leftMotor.setPower(-getTurnPower(angle, getAbsGyroHeading(), power));
				robot.rightMotor.setPower(getTurnPower(angle, getAbsGyroHeading(), power));

				telemetry.addData("Gyro", getAbsGyroHeading());
				telemetry.addData("left", getTurnPower(angle, getAbsGyroHeading(), power));
				telemetry.addData("right", -getTurnPower(angle, getAbsGyroHeading(), power));
				telemetry.update();
			}
		}

		stopDriveMotors();
	}

	private int getAbsGyroHeading()
	{
		if (robot.gyroSensor.getHeading() > 180)
		{
			return robot.gyroSensor.getHeading() - 360;
		}
		else
		{
			return robot.gyroSensor.getHeading();
		}
	}

	private double getTurnPower(double targetAngle, double heading, double power)
	{
		double powerAdjustment = 1 - (heading/targetAngle);
		double adjustedPower = powerAdjustment*power;

		if (targetAngle > 0)
		{
			return Range.clip(adjustedPower, robot.MINIMUM_SPEED, power);
		}
		else
		{
			return Range.clip(-adjustedPower, -power, -robot.MINIMUM_SPEED);
		}
	}

	private boolean isHeadingReached(double heading)
	{
		if (heading > 0)
		{
			return (getAbsGyroHeading() >= heading);
		}
		else
		{
			return (getAbsGyroHeading() <= heading);
		}
	}

	private double getPowerAdjustment()
	{
		return (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
	}

	private void stopDriveMotors()
	{
		robot.leftMotor.setPower(0);
		robot.rightMotor.setPower(0);

		robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}
}
