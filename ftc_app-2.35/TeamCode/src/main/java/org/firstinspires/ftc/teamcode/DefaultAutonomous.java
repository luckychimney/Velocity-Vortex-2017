package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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

		while (getVuforiaTrackableTranslation(robot.beacons) == null && opModeIsActive())
		{
			drive(robot.DEFAULT_DRIVE_SPEED, 775);
			sleep(1500);
		}

		beaconTranslation = getVuforiaTrackableTranslation(robot.beacons);
		distanceToDrive = getDistanceToDrive(beaconTranslation);
		degreesToTurn = getDegreesToTurn(beaconTranslation);

		telemetry.addData("Drive Distance", distanceToDrive);
		telemetry.addData("Turn Degrees", degreesToTurn);
		telemetry.update();

		turn(robot.DEFAULT_TURN_SPEED, degreesToTurn);
		drive(robot.DEFAULT_DRIVE_SPEED, distanceToDrive);
		turn(robot.DEFAULT_TURN_SPEED, -(90 + degreesToTurn));
		drive(robot.DEFAULT_DRIVE_SPEED, 440);
	}

	private int getDistanceToDrive(VectorF translation)
	{
		double distanceFromPhotoXAxesPlane = translation.get(0);
		double distanceFromWallBuffer = translation.get(2) - 450;

		return (int) Math.round(Math.sqrt(Math.pow(distanceFromPhotoXAxesPlane, 2) + Math.pow(Math.abs(distanceFromWallBuffer), 2)));
	}

	private int getDegreesToTurn(VectorF translation)
	{
		double distanceFromPhoto = translation.get(0);
		double distanceFromWall = translation.get(2);

		return (int) -Math.round(Math.abs(Math.toDegrees(Math.atan2(distanceFromPhoto, distanceFromWall))) - 90);
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
