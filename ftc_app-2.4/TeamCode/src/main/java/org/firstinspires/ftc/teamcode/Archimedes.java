package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

abstract class Archimedes extends LinearOpMode
{
	final double DEFAULT_DRIVE_SPEED = 0.65;
	final double DEFAULT_TURN_SPEED = 0.5;

	private final int ENCODER_UNITS_PER_REVOLUTION = 1440;
	private final double ENCODER_WHEEL_DIAMETER = 50.2;
	private final double ENCODER_UNITS_PER_MILLIMETER =
			(ENCODER_UNITS_PER_REVOLUTION /
					(ENCODER_WHEEL_DIAMETER * Math.PI));

	DcMotor leftMotor;
	DcMotor rightMotor;
	private DcMotor ballLauncher_;
	private DcMotor ballCollector_;
	private DcMotor liftMotor1;
	private DcMotor liftMotor2;

	private Servo ballDeployer_;
	private Servo capBallGrabber_;
	private Servo buttonPusher_;

	private ColorSensor colorSensor_;
	private ModernRoboticsI2cRangeSensor rangeSensor_;
	private OpticalDistanceSensor leftOpticalDistanceSensor_;
	private OpticalDistanceSensor rightOpticalDistanceSensor_;
	private GyroSensor gyroSensor_;

	private int lastRawGyroHeading_ = 0;
	private int gyroHeading_ = 0;
	private int targetGyroHeading_ = 0;
	private ElapsedTime elapsedTime_ = new ElapsedTime();

	void setLiftPower(double power)
	{
		liftMotor1.setPower(-power);
		liftMotor2.setPower(liftMotor1.getPower());
	}

	void startBallCollector()
	{
		ballCollector_.setPower(1);
	}

	void stopBallCollector()
	{
		ballCollector_.setPower(0);
	}

	void reverseBallCollector()
	{
		ballCollector_.setPower(-1);
	}

	void initializeArchimedes()
	{
		leftMotor = hardwareMap.dcMotor.get("left motor");
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightMotor = hardwareMap.dcMotor.get("right motor");
		rightMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		ballLauncher_ = hardwareMap.dcMotor.get("ball launcher");
		ballLauncher_.setDirection(DcMotor.Direction.REVERSE);
		ballLauncher_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		ballLauncher_.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		ballLauncher_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		ballCollector_ = hardwareMap.dcMotor.get("ball collector");
		ballCollector_.setDirection(DcMotor.Direction.REVERSE);
		ballCollector_.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		liftMotor1 = hardwareMap.dcMotor.get("lift motor 1");
		liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

		liftMotor2 = hardwareMap.dcMotor.get("lift motor 2");
		liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

		capBallGrabber_ = hardwareMap.servo.get("cap ball grabber");
		capBallGrabber_.setDirection(Servo.Direction.FORWARD);
		dropCapBallGrabber();

		buttonPusher_ = hardwareMap.servo.get("button pusher");
		buttonPusher_.setDirection(Servo.Direction.FORWARD);
		setButtonPusherToNeutral();

		ballDeployer_ = hardwareMap.servo.get("ball deployer");
		ballDeployer_.setDirection(Servo.Direction.FORWARD);
		dropBallDeployer();

		colorSensor_ = hardwareMap.colorSensor.get("color sensor");

		rangeSensor_ = hardwareMap
				.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

		leftOpticalDistanceSensor_ =
				hardwareMap.opticalDistanceSensor.get("left ODS");

		rightOpticalDistanceSensor_ =
				hardwareMap.opticalDistanceSensor.get("right ODS");

		gyroSensor_ = hardwareMap.gyroSensor.get("gyro");
		gyroSensor_.calibrate();
	}

	void dropCapBallGrabber()
	{
		capBallGrabber_.setPosition(0);
	}

	void setButtonPusherToNeutral()
	{
		buttonPusher_.setPosition(.3);
	}

	void dropBallDeployer()
	{
		ballDeployer_.setPosition(1);
	}

	void startBallLauncherForAutonomous()
	{
		ballLauncher_.setPower(0.55);
	}

	void startBallLauncherForTeleop()
	{
		ballLauncher_.setPower(.75);
	}

	void stopBallLauncher()
	{
		ballLauncher_.setPower(0.0);
	}

	void clampCapBallGrabber()
	{
		capBallGrabber_.setPosition(0.5);
	}

	void liftCapBallGrabber()
	{
		capBallGrabber_.setPosition(0.85);
	}

	void drive(double power, int distance)
	{
		double encoderUnitsToDrive = ENCODER_UNITS_PER_MILLIMETER * distance;

		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			getEncoderWheel().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			while (!isDriveDistanceReached(encoderUnitsToDrive) &&
					opModeIsActive())
			{
				leftAdjustedPower = Range.clip(
						getDrivePower(encoderUnitsToDrive, power) -
								getGyroPowerAdjustment(), -1, 1);
				rightAdjustedPower = Range.clip(
						getDrivePower(encoderUnitsToDrive, power) +
								getGyroPowerAdjustment(), -1, 1);
				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}
		}

		stopDriveMotors();
	}

	private boolean isDriveDistanceReached(double driveDistance)
	{
		return Math.abs(getEncoderWheel().getCurrentPosition()) <
				Math.abs(driveDistance);
	}

	private DcMotor getEncoderWheel()
	{
		return rightMotor;
	}

	private double getGyroPowerAdjustment()
	{
		final double GYRO_DRIVE_COEFFICIENT = 0.04;

		return (targetGyroHeading_ - getGyroHeading()) * GYRO_DRIVE_COEFFICIENT;
	}

	private int getGyroHeading()
	{
		int rawGyroHeading = gyroSensor_.getHeading();
		// 0->360
		if (rawGyroHeading - 180 > lastRawGyroHeading_)
		{
			gyroHeading_ =
					gyroHeading_ + rawGyroHeading - 360 -
							lastRawGyroHeading_;
		}
		// 359->0
		else if (rawGyroHeading + 180 < lastRawGyroHeading_)
		{
			gyroHeading_ =
					gyroHeading_ + rawGyroHeading + 360 -
							lastRawGyroHeading_;
		}
		else
		{
			gyroHeading_ =
					gyroHeading_ + rawGyroHeading - lastRawGyroHeading_;
		}

		lastRawGyroHeading_ = rawGyroHeading;
		return gyroHeading_;
	}

	private void stopDriveMotors()
	{
		leftMotor.setPower(0);
		rightMotor.setPower(0);

		getEncoderWheel().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	private double getDrivePower(double driveDistance, double power)
	{
		final double MINIMUM_DRIVE_POWER = .15;

		double adjustedPower = ((1 - MINIMUM_DRIVE_POWER) - (1 - power)) *
				Math.pow(Math.abs(
						getEncoderWheel().getCurrentPosition() / driveDistance),
						2) + MINIMUM_DRIVE_POWER;

		if (driveDistance > 0)
		{
			return Range.clip(adjustedPower, MINIMUM_DRIVE_POWER, power);
		}
		else
		{
			return Range.clip(-adjustedPower, -power, -MINIMUM_DRIVE_POWER);
		}
	}

	void driveToLine(double power, int maxDistance, double lightThreshold,
	                 int afterDriveDistance)
	{
		double encoderUnitsToDrive = ENCODER_UNITS_PER_MILLIMETER * maxDistance;
		double afterDriveEncoderUnitsToDrive = ENCODER_UNITS_PER_MILLIMETER *
				afterDriveDistance;
		double combinedDistance = encoderUnitsToDrive +
				afterDriveEncoderUnitsToDrive;

		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			getEncoderWheel().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			while (!isDriveDistanceReached(encoderUnitsToDrive) &&
					!isLineDetected(lightThreshold) && opModeIsActive())
			{
				leftAdjustedPower =
						Range.clip(getDrivePower(combinedDistance, power) -
								getGyroPowerAdjustment
										(), -1, 1);
				rightAdjustedPower =
						Range.clip(getDrivePower(combinedDistance, power) +
								getGyroPowerAdjustment(), -1, 1);

				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}

			while (!isDriveDistanceReached(combinedDistance) &&
					opModeIsActive())
			{
				leftAdjustedPower =
						Range.clip(getDrivePower(combinedDistance, power) -
								getGyroPowerAdjustment(), -1, 1);
				rightAdjustedPower =
						Range.clip(getDrivePower(combinedDistance, power) +
								getGyroPowerAdjustment(), -1, 1);

				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}
		}

		stopDriveMotors();
	}

	private boolean isLineDetected(double lightThreshold)
	{
		return leftOpticalDistanceSensor_.getLightDetected() > lightThreshold &&
				rightOpticalDistanceSensor_.getLightDetected() > lightThreshold;
	}

	void followBeaconLineToWall(double power, int maxDriveDistance,
	                            int distanceFromWall)
	{
		double encoderUnitsToDrive =
				ENCODER_UNITS_PER_MILLIMETER * maxDriveDistance;

		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			if (maxDriveDistance < 0)
			{
				power = power * -1;
			}

			while (!isDriveDistanceReached(encoderUnitsToDrive) &&
					!isWallInRangeThreshold(distanceFromWall)
					&& opModeIsActive())
			{
				leftAdjustedPower = Range.clip(
						power - getOdsPowerAdjustment(), -1,
						1);
				rightAdjustedPower = Range.clip(
						power + getOdsPowerAdjustment(), -1,
						1);
				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);
			}
		}

		stopDriveMotors();
	}

	private boolean isWallInRangeThreshold(int threshold)
	{
		return rangeSensor_.cmUltrasonic() > threshold;
	}

	private double getOdsPowerAdjustment()
	{
		final double OPTICAL_DISTANCE_SENSOR_COEFFICIENT = 0.65;

		return (rightOpticalDistanceSensor_.getLightDetected() -
				leftOpticalDistanceSensor_.getLightDetected()) *
				OPTICAL_DISTANCE_SENSOR_COEFFICIENT;
	}

	void turnButtonPusherRight()
	{
		buttonPusher_.setPosition(0);
	}

	void launchBall(int delay)
	{
		liftBallDeployer();
		sleep(delay);
		dropBallDeployer();
	}

	void liftBallDeployer()
	{
		ballDeployer_.setPosition(.75);
	}

	void waitForGyroCalibration()
	{
		telemetry.addData(">", "Calibrating gyro...");
		telemetry.update();

		sleep(1000);

		while (gyroSensor_.isCalibrating() && !isStopRequested())
		{
			idle();
			sleep(50);
		}

		telemetry.addData(">", "Archimedes ready!");
		telemetry.update();
	}

	boolean isDetectingBlueOnRight()
	{
		return colorSensor_.blue() > colorSensor_.red();
	}

	boolean isDetectingRedOnRight()
	{
		return colorSensor_.blue() < colorSensor_.red();
	}

	void turn(double power, int angleChange)
	{
		if (opModeIsActive())
		{
			getEncoderWheel().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			changeTargetGyroHeading(angleChange);

			while (!isHeadingReached(angleChange) && opModeIsActive())
			{
				double turnPower = getTurnPower(angleChange, power);
				leftMotor.setPower(-turnPower);
				rightMotor.setPower(turnPower);
			}
		}

		stopDriveMotors();
	}

	private boolean isHeadingReached(double heading)
	{
		if (heading > 0)
		{
			return (getGyroHeading() > targetGyroHeading_);
		}
		else
		{
			return (getGyroHeading() < targetGyroHeading_);
		}
	}

	private double getTurnPower(double angleChange, double power)
	{
		final double MINIMUM_TURN_POWER = .35;

		double adjustedPower = ((1 - MINIMUM_TURN_POWER) - (1 - power)) *
				Math.pow(1 - (Math.abs(
						targetGyroHeading_ - getGyroHeading() / angleChange)),
						2) + MINIMUM_TURN_POWER;

		if (angleChange > 0)
		{
			return Range.clip(adjustedPower, MINIMUM_TURN_POWER, power);
		}
		else
		{
			return Range.clip(-adjustedPower, -power, -MINIMUM_TURN_POWER);
		}
	}

	private void changeTargetGyroHeading(int headingChange)
	{
		targetGyroHeading_ += headingChange;
	}

	void turnButtonPusherLeft()
	{
		buttonPusher_.setPosition(.6);
	}

	void timeDrive(double power, int time)
	{
		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			leftMotor.setPower(power);
			rightMotor.setPower(power);

			double startTime = elapsedTime_.milliseconds();

			while (startTime + time > elapsedTime_.milliseconds() &&
					opModeIsActive())
			{
				sleep(50);
				idle();
			}
		}

		stopDriveMotors();
	}
}
