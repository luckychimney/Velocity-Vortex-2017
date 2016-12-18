package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

abstract public class Robot extends LinearOpMode
{
	final double DEFAULT_DRIVE_SPEED = 1.0;
	final double DEFAULT_TURN_SPEED = 0.75;

	DcMotor leftMotor;
	DcMotor rightMotor;
	DcMotor ballLauncher;
	DcMotor ballCollector;
	OpticalDistanceSensor leftOpticalDistanceSensor;
	OpticalDistanceSensor rightOpticalDistanceSensor;
	Servo ballDeployer;
	GyroSensor gyroSensor;

	private int lastRawGyroHeading = 0;
	private int absGyroHeading = 0;
	private int targetGyroHeading = 0;
	private ElapsedTime elapsedTime = new ElapsedTime();

	void initializeRobot()
	{
		leftMotor = hardwareMap.dcMotor.get("left motor");
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightMotor = hardwareMap.dcMotor.get("right motor");
		rightMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		/*ballLauncher = hardwareMap.dcMotor.get("ball launcher");
		ballLauncher.setDirection(DcMotor.Direction.REVERSE);
		ballLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		ballLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		ballLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		ballCollector = hardwareMap.dcMotor.get("ball collector");
		ballCollector.setDirection(DcMotor.Direction.REVERSE);
		ballCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		ballDeployer = hardwareMap.servo.get("ball deployer");
		ballDeployer.setDirection(Servo.Direction.FORWARD);
		ballDeployer.setPosition(1);*/

		//TODO Add these back

		leftOpticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("left ODS");

		rightOpticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("right ODS");

		gyroSensor = hardwareMap.gyroSensor.get("gyro");
		gyroSensor.calibrate();
	}

	void drive(double power, int distance)
	{
		final int encoderUnitsPerRevolution = 1440;
		final double encoderWheelDiameter = 50.2;
		final double encoderUnitsPerMillimeter = (encoderUnitsPerRevolution / (encoderWheelDiameter * Math.PI));
		//final double driveStartTime = elapsedTime.seconds();

		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			double encoderUnitsToDrive = encoderUnitsPerMillimeter * distance;

			if (distance < 0)
			{
				power = power * -1;
			}

			while ((Math.abs(getEncoderWheel().getCurrentPosition()) < Math.abs(encoderUnitsToDrive) && opModeIsActive()))
			{
				//TODO Chnage the power adjustment back to gyor
				leftAdjustedPower = Range.clip(power - getOpticalDistanceSensorPowerAdjustment(), -1, 1);
				rightAdjustedPower = Range.clip(power + getOpticalDistanceSensorPowerAdjustment(), -1, 1);
				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);

				telemetry.addData("Angle", getAbsGyroHeading());
				telemetry.addData("Target", targetGyroHeading);

				telemetry.addData("Adjusted Power", getGyroPowerAdjustment());
				telemetry.addData("Encoder", getEncoderWheel().getCurrentPosition());
				telemetry.update();
			}
		}

		stopDriveMotors();
	}

	private DcMotor getEncoderWheel()
	{
		return rightMotor;
	}

	double getOpticalDistanceSensorPowerAdjustment()
	{
		final double opticalDistanceSensorCoefficient = .4;

		return (leftOpticalDistanceSensor.getLightDetected() - rightOpticalDistanceSensor.getLightDetected()) * opticalDistanceSensorCoefficient;
	}

	double getAbsGyroHeading()
	{
		int gyroHeading = gyroSensor.getHeading();
		//359<-0
		if (gyroHeading - 180 > lastRawGyroHeading)
		{
			absGyroHeading = absGyroHeading + gyroHeading - 360 - lastRawGyroHeading;
		}
		//359->0
		else if (gyroHeading + 180 < lastRawGyroHeading)
		{
			absGyroHeading = absGyroHeading + gyroHeading + 360 - lastRawGyroHeading;
		}
		else
		{
			absGyroHeading = absGyroHeading + gyroHeading - lastRawGyroHeading;
		}

		lastRawGyroHeading = gyroHeading;
		return absGyroHeading;
	}

	double getGyroPowerAdjustment()
	{
		final double gyroDriveCoefficient = 0.015;

		return (getAbsGyroHeading() - targetGyroHeading) * gyroDriveCoefficient;
	}

	private void stopDriveMotors()
	{
		leftMotor.setPower(0);
		rightMotor.setPower(0);

		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	double getCombinedPowerAdjustment()
	{
		double adjustment;

		adjustment = Math.sqrt(Math.pow(getGyroPowerAdjustment(), 2) + Math.pow(getOpticalDistanceSensorPowerAdjustment(), 2));

		if (getGyroPowerAdjustment() > 0)
		{
			return adjustment;
		}
		else if (getGyroPowerAdjustment() < 0)
		{
			return -adjustment;
		}
		else
		{
			if (getGyroPowerAdjustment() >= 0)
			{
				return adjustment;
			}
			else
			{
				return -adjustment;
			}
		}
	}

	/*private boolean isTiltDetected(double driveStartTime)
	{
		return (elapsedTime.seconds() > driveStartTime + 2 && gyroSensor.rawX() > 1000);
	}*/

	void launchBall(int delay)
	{
		liftBallDeployer();
		sleep(delay);
		dropBallDeployer();
	}

	private void liftBallDeployer()
	{
		ballDeployer.setPosition(.65);
	}

	private void dropBallDeployer()
	{
		ballDeployer.setPosition(1);
	}

	void waitForGyroCalibration()
	{
		telemetry.addData(">", "Calibrating gyro...");
		telemetry.update();

		sleep(1000);

		while (gyroSensor.isCalibrating() && !isStopRequested())
		{
			idle();
			sleep(50);
		}

		telemetry.addData(">", "Robot ready!");
		telemetry.update();
	}

	void turn(double power, int angle)
	{
		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			targetGyroHeading += angle;

			while (!isHeadingReached(angle) && opModeIsActive())
			{
				leftMotor.setPower(-getTurnPower(angle, getAbsGyroHeading(), power));
				rightMotor.setPower(getTurnPower(angle, getAbsGyroHeading(), power));

				telemetry.addData("Gyro", getAbsGyroHeading());
				telemetry.addData("Target", targetGyroHeading);
				telemetry.addData("left", getTurnPower(angle, getAbsGyroHeading(), power));
				telemetry.addData("right", -getTurnPower(angle, getAbsGyroHeading(), power));
				telemetry.update();
			}
		}

		stopDriveMotors();
	}

	private boolean isHeadingReached(double heading)
	{
		if (heading > 0)
		{
			return (getAbsGyroHeading() > targetGyroHeading);
		}
		else
		{
			return (getAbsGyroHeading() < targetGyroHeading);
		}
	}

	private double getTurnPower(double targetAngle, double heading, double power)
	{
		final double minimumSpeed = .27;

		double powerAdjustment = 1 - Math.abs(heading / targetGyroHeading);
		double adjustedPower = powerAdjustment * power;

		if (targetAngle - heading > 0)
		{
			return Range.clip(adjustedPower, minimumSpeed, power);
		}
		else
		{
			return Range.clip(-adjustedPower, -power, -minimumSpeed);
		}
	}

	void timeDrive(double power, int time)
	{
		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			leftMotor.setPower(power);
			rightMotor.setPower(power);

			double startTime = elapsedTime.milliseconds();

			while (startTime + time > elapsedTime.milliseconds() && opModeIsActive())
			{
				sleep(50);
				idle();
			}
		}

		stopDriveMotors();
	}
}
