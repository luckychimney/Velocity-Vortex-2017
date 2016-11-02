package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Autonomous")
public class DefaultAutonomous extends LinearOpMode
{
	private Robot robot = new Robot();

	@Override
	public void runOpMode() throws InterruptedException
	{
		int degreesToTurn;
		int distanceToDrive;
		VectorF beaconTranslation;

		robot.init(hardwareMap);

		while (robot.gyroSensor.isCalibrating())
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

		while (getVuforiaTrackableTranslation(robot.beacons) == null && opModeIsActive())
		{
			drive(0.25, 150);
			sleep(1500);
		}

		beaconTranslation = getVuforiaTrackableTranslation(robot.beacons);

		distanceToDrive = getDistanceToDrive(beaconTranslation);
		degreesToTurn = getDegreesToTurn(beaconTranslation);

		turn(0.25, -degreesToTurn);

		drive(0.25, distanceToDrive);

		turn(0.25, -(90 - degreesToTurn));

		drive(0.25, 450);
	}

	private int getDistanceToDrive(VectorF translation)
	{
		double x = translation.get(0);
		double z = translation.get(2);

		return (int) Math.round(Math.sqrt(Math.pow(x, 2) + Math.pow(Math.abs(z), 2)));
	}

	private int getDegreesToTurn(VectorF translation)
	{
		double distanceFromPhoto = translation.get(0);
		double distanceFromWall = translation.get(2);

		return (int) Math.round(Math.abs(
				Math.toDegrees(Math.atan2(distanceFromPhoto, distanceFromWall))) - 90);
	}

	@Nullable
	private VectorF getVuforiaTrackableTranslation(VuforiaTrackables beacons)
	{
		for (VuforiaTrackable beacon : beacons)
		{
			OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

			if (pose != null)
			{
				return pose.getTranslation();
			}
		}
		return null;
	}

	private void drive(double power, int distance)
	{
		double leftAdjustedPower;
		double rightAdjustedPower;

		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		double encoderUnitsToDrive = robot.ENCODER_UNITS_PER_MILLIMETER * distance;

		robot.gyroSensor.resetZAxisIntegrator();

		if (distance > 0)
		{
			while (robot.rightMotor.getCurrentPosition() < encoderUnitsToDrive && opModeIsActive())
			{
				leftAdjustedPower = power - (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
				rightAdjustedPower = power + (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
				robot.leftMotor.setPower(leftAdjustedPower);
				robot.rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}
		}
		else
		{
			while (robot.leftMotor.getCurrentPosition() > encoderUnitsToDrive && opModeIsActive())
			{
				leftAdjustedPower = power + (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
				rightAdjustedPower = power - (getAbsGyroHeading() * robot.GYRO_DRIVE_COEFFICIENT);
				robot.leftMotor.setPower(leftAdjustedPower);
				robot.rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}
		}

		stopDriveMotors();
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
