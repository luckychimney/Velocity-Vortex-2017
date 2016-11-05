package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Basic Autonomous")
public class ForwardAutonomous extends LinearOpMode
{
	private Robot robot = new Robot();

	@Override
	public void runOpMode() throws InterruptedException
	{
		robot.init(hardwareMap);

		while (robot.gyroSensor.isCalibrating() & opModeIsActive())
		{
			telemetry.addData(">", "Calibrating gyro...");
			telemetry.update();
			idle();
			sleep(50);
		}

		telemetry.addData(">", "Robot ready!");
		telemetry.update();

		waitForStart();

		telemetry.addData(">", "Robot running...");
		telemetry.update();

		drive(0.4, 780);
		turn(0.5, -45);
		turn(0.5, 45);
		drive(0.4, 75);
	}

	private void drive(double power, int distance)
	{
		double leftAdjustedPower;
		double rightAdjustedPower;

		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		double encoderUnitsToDrive = robot.ENCODER_UNITS_PER_MILLIMETER * distance;

		robot.gyroSensor.resetZAxisIntegrator();

		while (Math.abs(robot.getEncoderWheel().getCurrentPosition()) < encoderUnitsToDrive && opModeIsActive())
		{
			leftAdjustedPower = Range.clip(power - getPowerAdjustment(), 0, 1);
			rightAdjustedPower = Range.clip(power + getPowerAdjustment(), 0, 1);
			robot.leftMotor.setPower(leftAdjustedPower);
			robot.rightMotor.setPower(rightAdjustedPower);
			idle();
			sleep(50);
		}

		stopDriveMotors();
	}

	private double getPowerAdjustment()
	{
		if (robot.rightMotor.getPower() > 0)
		{
			return (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
		}
		else
		{
			return -(getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
		}
	}

	private void stopDriveMotors()
	{
		robot.leftMotor.setPower(0);
		robot.rightMotor.setPower(0);

		robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

	private void turn(double power, int degrees)
	{
		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		robot.gyroSensor.resetZAxisIntegrator();

		if (degrees > 0)
		{
			robot.leftMotor.setPower(power);
			robot.rightMotor.setPower(-power);
		}
		else
		{
			robot.leftMotor.setPower(-power);
			robot.rightMotor.setPower(power);
		}

		while (!isHeadingReached(degrees) && opModeIsActive())
		{
			idle();
			sleep(50);
		}

		stopDriveMotors();
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
}
