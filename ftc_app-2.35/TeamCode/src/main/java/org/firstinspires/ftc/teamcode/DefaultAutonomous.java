package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Default Autonomous")
@Disabled
public class DefaultAutonomous extends LinearOpMode
{
	private Robot robot = new Robot();

	private ElapsedTime elapsedTime = new ElapsedTime();

	@Override
	public void runOpMode() throws InterruptedException
	{
		int degreesToTurn;
		int distanceToDrive;
		VectorF beaconTranslation;

		robot.init(hardwareMap);

		while (!robot.gyroSensor.isCalibrating())
		{
			telemetry.addData(">", "Waiting for gyro calibration...");
			telemetry.update();
			idle();
			sleep(50);
		}

		while (robot.gyroSensor.isCalibrating() && opModeIsActive())
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

		drive(robot.DEFAULT_DRIVE_SPEED, 800);

		while (getVuforiaTrackableTranslation(robot.beacons) == null && opModeIsActive())
		{
			drive(robot.DEFAULT_DRIVE_SPEED, 20);
			sleep(1500);
		}

		beaconTranslation = getVuforiaTrackableTranslation(robot.beacons);
		distanceToDrive = getDistanceToDrive(beaconTranslation) / 4;
		degreesToTurn = getDegreesToTurn(beaconTranslation) + 8;

		telemetry.addData("Drive Distance", distanceToDrive);
		telemetry.addData("Turn Degrees", degreesToTurn);
		telemetry.update();

		turn(robot.DEFAULT_TURN_SPEED, degreesToTurn);
		drive(robot.DEFAULT_DRIVE_SPEED, distanceToDrive);
		//turn(robot.DEFAULT_TURN_SPEED, -(90 + degreesToTurn));
		//drive(robot.DEFAULT_DRIVE_SPEED, 440);

	}

	private int getDistanceToDrive(VectorF translation)
	{
		double distanceFromImageXAxesPlane = translation.get(0);
		double distanceFromWallBuffer = translation.get(2) - robot.WALL_BUFFER;

		return (int) Math.round(Math.sqrt(Math.pow(distanceFromImageXAxesPlane, 2) + Math.pow(Math.abs(distanceFromWallBuffer), 2)));
	}

	@SuppressWarnings("SuspiciousNameCombination")
	private int getDegreesToTurn(VectorF translation)
	{
		double distanceFromImageXAxesPlane = translation.get(0);
		double distanceFromWallBuffer = translation.get(2) - robot.WALL_BUFFER;

		return (int) -Math.round(Math.abs(Math.toDegrees(Math.atan2(distanceFromImageXAxesPlane, distanceFromWallBuffer))) - 90);
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

	private void timeDrive(double power, int time)
	{
		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		robot.leftMotor.setPower(power);
		robot.rightMotor.setPower(power);

		double startTime = elapsedTime.milliseconds();

		while (startTime + time > elapsedTime.milliseconds())
		{
			sleep(50);
			idle();
		}

		stopDriveMotors();
	}

	private void drive(double power, int distance)
	{
		double leftAdjustedPower;
		double rightAdjustedPower;

		robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		double encoderUnitsToDrive = robot.ENCODER_UNITS_PER_MILLIMETER * distance;

		robot.gyroSensor.resetZAxisIntegrator();

		if (distance < 0)
		{
			power = power*-1;
		}

		while((Math.abs(robot.getEncoderWheel().getCurrentPosition()) < Math.abs(encoderUnitsToDrive) && opModeIsActive()))
		{
			leftAdjustedPower = Range.clip(power - getPowerAdjustment(distance), -1, 1);
			rightAdjustedPower = Range.clip(power + getPowerAdjustment(distance), -1, 1);
			robot.leftMotor.setPower(leftAdjustedPower);
			robot.rightMotor.setPower(rightAdjustedPower);
			idle();
			sleep(50);

			telemetry.addData("Angle", getAbsGyroHeading());
			telemetry.addData("Adjusted Power", getPowerAdjustment(distance));
			telemetry.addData("Encoder", robot.getEncoderWheel().getCurrentPosition());
			telemetry.update();
		}

		stopDriveMotors();
	}

	private double getPowerAdjustment(int distance)
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

	private double getTurnError(double target, double heading)
	{
		return ((target - heading)/100) * robot.TURN_COEFFICIENT;
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

	private void turn(double power, int angle)
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
