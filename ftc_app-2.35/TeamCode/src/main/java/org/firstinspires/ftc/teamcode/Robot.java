package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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
	Servo ballDeployer;
	GyroSensor gyroSensor;

	private int lastRawGyroHeading = 0;
	private int absGyroHeading = 0;
	private int targetGyroHeading = 0;
	private ElapsedTime elapsedTime = new ElapsedTime();

//	VuforiaTrackables beacons;

	void initializeRobot()
	{
		leftMotor = hardwareMap.dcMotor.get("left motor");
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightMotor = hardwareMap.dcMotor.get("right motor");
		rightMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		ballLauncher = hardwareMap.dcMotor.get("ball launcher");
		ballLauncher.setDirection(DcMotor.Direction.REVERSE);
		ballLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		ballLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		ballLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		ballCollector = hardwareMap.dcMotor.get("ball collector");
		ballCollector.setDirection(DcMotor.Direction.REVERSE);
		ballCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		ballDeployer = hardwareMap.servo.get("ball deployer");
		ballDeployer.setDirection(Servo.Direction.FORWARD);
		ballDeployer.setPosition(1);

//		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//		parameters.vuforiaLicenseKey = "AVjSL4f/////AAAAGRrqOmjjwUvcra1uL+pn/W8AoLn03Yj7g6Aw+VGRAI+CkzXWFw/7FLW09TYRSzxCcQmlWovvlsq9k4DqqxDr+bnAVhsmk+MNEzKyMBqkwMM6BGjEL6ohtkGbMiE+sYL0aWgZ+UL
// u6pPJZQboiH/sEcH2jq8o5zAVe3lbP9E34gCELlHAIzgEta7lXabdjC86OixIDZbdEBpE5UTGPRFKTbYgKFVNoouFgUT4hs5MiqD21DwbubgmSe+WOVyi3G4WTkJowT9jx1XlOrUXwc6kfyArQ+DFQNXEghwAXhC9FOEWijQTKSG+TDq7XePfRICqLPE
// dl4aYUixHn6OCCPZ85o7bEaBYf74ZddKg7IBTCOsg";
//		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
//		Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
//		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

//		beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
//		beacons.get(0).setName("Wheels");
//		beacons.get(1).setName("Tools");
//		beacons.get(2).setName("Lego");
//		beacons.get(3).setName("Gears");
//		beacons.activate();

		gyroSensor = hardwareMap.gyroSensor.get("gyro");
		gyroSensor.calibrate();
	}

	void drive(double power, int distance)
	{
		final int encoderUnitsPerRevolution = 1440;
		final double encoderWheelDiameter = 50.2;
		final double encoderUnitsPerMillimeter = (encoderUnitsPerRevolution / (encoderWheelDiameter * Math.PI));
		final double driveStartTime = elapsedTime.seconds();

		double leftAdjustedPower;
		double rightAdjustedPower;

		if (opModeIsActive())
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			double encoderUnitsToDrive = encoderUnitsPerMillimeter * distance;

			gyroSensor.resetZAxisIntegrator();

			if (distance < 0)
			{
				power = power * -1;
			}

			while ((Math.abs(getEncoderWheel().getCurrentPosition()) < Math.abs(encoderUnitsToDrive) && opModeIsActive()))
			{
				leftAdjustedPower = Range.clip(power - getPowerAdjustment(), -1, 1);
				rightAdjustedPower = Range.clip(power + getPowerAdjustment(), -1, 1);
				leftMotor.setPower(leftAdjustedPower);
				rightMotor.setPower(rightAdjustedPower);
				idle();
				sleep(50);

				telemetry.addData("Angle", getAbsGyroHeading());
				telemetry.addData("Target", targetGyroHeading);

				telemetry.addData("Adjusted Power", getPowerAdjustment());
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

	private double getPowerAdjustment()
	{
		final double GYRO_DRIVE_COEFFICIENT = 0.015;

		return (targetGyroHeading - getAbsGyroHeading()) * GYRO_DRIVE_COEFFICIENT;
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

	private void stopDriveMotors()
	{
		leftMotor.setPower(0);
		rightMotor.setPower(0);

		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	private boolean isTiltDetected(double driveStartTime)
	{
		return (elapsedTime.seconds() > driveStartTime + 2 && gyroSensor.rawX() > 1000);
	}

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
		final double MINIMUM_SPEED = .27;

		double powerAdjustment = 1 - Math.abs(heading / targetGyroHeading);
		double adjustedPower = powerAdjustment * power;

		if (targetAngle > 0)
		{
			return Range.clip(adjustedPower, MINIMUM_SPEED, power);
		}
		else
		{
			return Range.clip(-adjustedPower, -power, -MINIMUM_SPEED);
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
