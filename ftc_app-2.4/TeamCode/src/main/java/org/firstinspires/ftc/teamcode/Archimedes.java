package org.firstinspires.ftc.teamcode;import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.ColorSensor;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.DcMotorSimple;import com.qualcomm.robotcore.hardware.GyroSensor;import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;import com.qualcomm.robotcore.hardware.Servo;import com.qualcomm.robotcore.util.ElapsedTime;import com.qualcomm.robotcore.util.Range;/** * The Archimedes class contains all the methods that run the robot for the * 2016-2017 robot for the FTC team #5037 got robot? * * @author got robot? * @version 2.2 * @since 2017-01-23 */abstract class Archimedes extends LinearOpMode{	final double DEFAULT_DRIVE_POWER = 1.0;	final double DEFAULT_TURN_POWER = 0.85;	final double MINIMUM_DRIVE_POWER = 0.15;	final double MINIMUM_TURN_POWER = 0.35;	private final int ENCODER_UNITS_PER_REVOLUTION = 1440;	private final double ENCODER_WHEEL_DIAMETER = 50.2;	private final double ENCODER_UNITS_PER_MILLIMETER =			(ENCODER_UNITS_PER_REVOLUTION /					(ENCODER_WHEEL_DIAMETER * Math.PI));	private int lastRawGyroHeading_ = 0;	private int gyroHeading_ = 0;	private int targetGyroHeading_ = 0;	private ElapsedTime elapsedTime = new ElapsedTime();	private DcMotor leftDriveMotor;	private DcMotor rightDriveMotor;	private DcMotor ballLauncher;	private DcMotor ballCollector;	private DcMotor liftMotor1;	private DcMotor liftMotor2;	private Servo ballDeployer;	private Servo capBallGrabber;	private Servo buttonPusher;	private ColorSensor colorSensor;	private ModernRoboticsI2cRangeSensor rangeSensor;	private OpticalDistanceSensor leftOds;	private OpticalDistanceSensor rightOds;	private GyroSensor gyroSensor;	void startArchimedes()	{		if (opModeIsActive())		{			leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			telemetry.addData(">", "Archimedes running...");			telemetry.update();		}	}	DcMotor getLeftDriveMotor()	{		return leftDriveMotor;	}	DcMotor getRightDriveMotor()	{		return rightDriveMotor;	}	/**	 * Sets the power for both lift motors	 *	 * @param power The power the lift should run at.	 */	void setLiftPower(double power)	{		if (opModeIsActive())		{			liftMotor1.setPower(-power);			liftMotor2.setPower(liftMotor1.getPower());		}	}	void startBallCollector()	{		if (opModeIsActive())			ballCollector.setPower(1);	}	void stopBallCollector()	{		if (opModeIsActive())			ballCollector.setPower(0);	}	void reverseBallCollector()	{		if (opModeIsActive())			ballCollector.setPower(-1);	}	/**	 * Initializes all of the motors, servos and sensors. Also beings	 * calibrating the gyro sensor.	 */	void initializeArchimedes()	{		leftDriveMotor = hardwareMap.dcMotor.get("left motor");		leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);		leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);		rightDriveMotor = hardwareMap.dcMotor.get("right motor");		rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);		rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);		ballLauncher = hardwareMap.dcMotor.get("ball launcher");		ballLauncher.setDirection(DcMotor.Direction.REVERSE);		ballLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);		ballLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);		ballLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);		ballCollector = hardwareMap.dcMotor.get("ball collector");		ballCollector.setDirection(DcMotor.Direction.REVERSE);		ballCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);		liftMotor1 = hardwareMap.dcMotor.get("lift motor 1");		liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);		liftMotor2 = hardwareMap.dcMotor.get("lift motor 2");		liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);		capBallGrabber = hardwareMap.servo.get("cap ball grabber");		capBallGrabber.setDirection(Servo.Direction.FORWARD);		dropCapBallGrabber();		buttonPusher = hardwareMap.servo.get("button pusher");		buttonPusher.setDirection(Servo.Direction.FORWARD);		setButtonPusherToNeutral();		ballDeployer = hardwareMap.servo.get("ball deployer");		ballDeployer.setDirection(Servo.Direction.FORWARD);		dropBallDeployer();		colorSensor = hardwareMap.colorSensor.get("color sensor");		rangeSensor = hardwareMap				.get(ModernRoboticsI2cRangeSensor.class, "range sensor");		leftOds =				hardwareMap.opticalDistanceSensor.get("left ODS");		rightOds =				hardwareMap.opticalDistanceSensor.get("right ODS");		gyroSensor = hardwareMap.gyroSensor.get("gyro");		gyroSensor.calibrate();	}	void dropCapBallGrabber()	{		if (opModeIsActive())			capBallGrabber.setPosition(0);	}	void setButtonPusherToNeutral()	{		if (opModeIsActive())			buttonPusher.setPosition(.3);	}	void dropBallDeployer()	{		if (opModeIsActive())			ballDeployer.setPosition(1);	}	void startBallLauncherForAutonomous()	{		if (opModeIsActive())			ballLauncher.setPower(0.55);	}	void startBallLauncherForTeleop()	{		if (opModeIsActive())			ballLauncher.setPower(.75);	}	void stopBallLauncher()	{		ballLauncher.setPower(0.0);	}	void clampCapBallGrabber()	{		if (opModeIsActive())			capBallGrabber.setPosition(0.5);	}	void liftCapBallGrabber()	{		if (opModeIsActive())			capBallGrabber.setPosition(0.85);	}	/**	 * Tells the robot to drive forward a set distance. Slows down as it gets	 * closer to the set distance. Also corrects its self if using the gyro	 * sensor if it starts to drift.	 *	 * @param power    The maximum power the robot will drive.	 * @param distance The distance the robot should drive in cm.	 *	 * @see #getGyroPowerAdjustment()	 * @see #getDrivePower(double, double)	 */	void drive(double power, int distance)	{		final double encoderUnitsToDrive = ENCODER_UNITS_PER_MILLIMETER *				distance;		double leftAdjustedPower;		double rightAdjustedPower;		if (opModeIsActive())		{			leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			while (!isDriveDistanceReached(encoderUnitsToDrive) &&					opModeIsActive())			{				leftAdjustedPower = Range.clip(						getDrivePower(encoderUnitsToDrive, power) -								getGyroPowerAdjustment(), -1, 1);				rightAdjustedPower = Range.clip(						getDrivePower(encoderUnitsToDrive, power) +								getGyroPowerAdjustment(), -1, 1);				leftDriveMotor.setPower(leftAdjustedPower);				rightDriveMotor.setPower(rightAdjustedPower);				idle();				sleep(50);			}		}		stopDriveMotors();	}	/**	 * Tells when the distace the robot was driving for was reached.	 *	 * @param driveDistance The target distance the robot was driving for.	 *	 * @return True if the distance has been reached.	 */	private boolean isDriveDistanceReached(double driveDistance)	{		return Math.abs(getEncoderWheel().getCurrentPosition()) >=				Math.abs(driveDistance);	}	private DcMotor getEncoderWheel()	{		return rightDriveMotor;	}	private double getGyroPowerAdjustment()	{		final double GYRO_DRIVE_COEFFICIENT = 0.04;		return (targetGyroHeading_ - getGyroHeading()) * GYRO_DRIVE_COEFFICIENT;	}	/**	 * Calculate the gyro headingby taking the raw gyro heading taken from	 * the sensor and converting it into a more usable value.	 *	 * @return The gyro heading.	 */	private int getGyroHeading()	{		int rawGyroHeading = gyroSensor.getHeading();		// 0->360		if (rawGyroHeading - 180 > lastRawGyroHeading_)		{			gyroHeading_ =					gyroHeading_ + rawGyroHeading - 360 -							lastRawGyroHeading_;		}		// 359->0		else if (rawGyroHeading + 180 < lastRawGyroHeading_)		{			gyroHeading_ =					gyroHeading_ + rawGyroHeading + 360 -							lastRawGyroHeading_;		}		else		{			gyroHeading_ =					gyroHeading_ + rawGyroHeading - lastRawGyroHeading_;		}		lastRawGyroHeading_ = rawGyroHeading;		return gyroHeading_;	}	/**	 * Stops both drive motors and resets the encoder wheel.	 */	private void stopDriveMotors()	{		leftDriveMotor.setPower(0);		rightDriveMotor.setPower(0);		getEncoderWheel().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);	}	/**	 * Generates a parabolic function and slows the robot down while driving	 * forwards or backwards using the parameters.	 *	 * @param driveDistance The distance that the robot is attempting to drive.	 * @param power         The maximum power the robot should drive at.	 *	 * @return The power that the robot should drive at.	 */	private double getDrivePower(double driveDistance, double power)	{		double adjustedPower = -((1 - MINIMUM_DRIVE_POWER) - (1 - power)) *				Math.pow(Math.abs(getEncoderWheel().getCurrentPosition()) /						Math.abs(driveDistance), 2) + power;		if (driveDistance > 0)		{			return Range.clip(adjustedPower, MINIMUM_DRIVE_POWER, power);		}		else		{			return Range.clip(-adjustedPower, -power, -MINIMUM_DRIVE_POWER);		}	}	/**	 * Drives the robot until it finds a line and then drives slightly	 * farther. Slows down as it gets closer to the point where the line is	 * expected. Also uses gyro correction to correct the drive if it drifts.	 *	 * @param power              The maximum power the robot should drive at	 * @param maxDistance        The maximum distance the robot should drive before	 *                           stopping.	 * @param lightThreshold     The threshold in which the robot detects a line	 *                           and stops.	 * @param afterDriveDistance The distance the robot should drive after it	 *                           finds the line.	 *	 * @see #getGyroPowerAdjustment()	 * @see #getDrivePower(double, double)	 */	void driveToLine(double power, int maxDistance, double lightThreshold,	                 int afterDriveDistance)	{		final double encoderUnitsToDrive = ENCODER_UNITS_PER_MILLIMETER *				maxDistance;		final double afterDriveEncoderUnitsToDrive =				ENCODER_UNITS_PER_MILLIMETER *						afterDriveDistance;		final double combinedDistance = encoderUnitsToDrive +				afterDriveEncoderUnitsToDrive;		double leftAdjustedPower;		double rightAdjustedPower;		if (opModeIsActive())		{			getEncoderWheel().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			while (!isDriveDistanceReached(encoderUnitsToDrive) &&					!isLineDetected(lightThreshold) && opModeIsActive())			{				leftAdjustedPower =						Range.clip(getDrivePower(combinedDistance, power) -								getGyroPowerAdjustment										(), -1, 1);				rightAdjustedPower =						Range.clip(getDrivePower(combinedDistance, power) +								getGyroPowerAdjustment(), -1, 1);				leftDriveMotor.setPower(leftAdjustedPower);				rightDriveMotor.setPower(rightAdjustedPower);				idle();				sleep(50);			}			while (!isDriveDistanceReached(combinedDistance) &&					opModeIsActive())			{				leftAdjustedPower =						Range.clip(getDrivePower(combinedDistance, power) -								getGyroPowerAdjustment(), -1, 1);				rightAdjustedPower =						Range.clip(getDrivePower(combinedDistance, power) +								getGyroPowerAdjustment(), -1, 1);				leftDriveMotor.setPower(leftAdjustedPower);				rightDriveMotor.setPower(rightAdjustedPower);				idle();				sleep(50);			}		}		stopDriveMotors();	}	/**	 * Tells if a light has been detected by the optical distance sensors. In	 * almost every case this will be used for detecting the lines in front	 * of the beacons.	 *	 * @param lightThreshold The light threshold that a line has been detected.	 *	 * @return True if a line has been detected.	 */	private boolean isLineDetected(double lightThreshold)	{		return leftOds.getLightDetected() > lightThreshold &&				rightOds.getLightDetected() > lightThreshold;	}	/**	 * Follows the line that is in front of the beacons up to the wall. Slows	 * down as it gets closer to the wall.	 *	 * @param power            The maximum power the robot should drive at.	 * @param distanceFromWall The distance from the wall the robot should	 *                         stop at.	 *	 * @see #getOdsPowerAdjustment()	 * @see #getRangeDrivePower(double, double)	 */	void followBeaconLineToWall(double power, int distanceFromWall)	{		double leftAdjustedPower;		double rightAdjustedPower;		if (opModeIsActive())		{			final double startingDistanceFromWall =					rangeSensor.cmUltrasonic();			leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			while (!isWallInRangeThreshold(distanceFromWall)					&& opModeIsActive())			{				// TODO Test this part of the function				leftAdjustedPower = Range.clip(						getRangeDrivePower(startingDistanceFromWall,								power) - getOdsPowerAdjustment(), -1, 1);				rightAdjustedPower = Range.clip(						getRangeDrivePower(startingDistanceFromWall,								power) + getOdsPowerAdjustment(), -1, 1);				leftDriveMotor.setPower(leftAdjustedPower);				rightDriveMotor.setPower(rightAdjustedPower);				idle();				sleep(50);			}		}		stopDriveMotors();	}	private double getOdsPowerAdjustment()	{		final double OPTICAL_DISTANCE_SENSOR_COEFFICIENT = 0.65;		return (rightOds.getLightDetected() -				leftOds.getLightDetected()) *				OPTICAL_DISTANCE_SENSOR_COEFFICIENT;	}	/**	 * Creates a parabolic function using the range sensor so that the robot	 * slows down as it gets closer to the wall.	 *	 * @param startingDistanceFromWall The distance we started at the wall from	 * @param power                    The maximum power the robot should drive at	 *	 * @return The power should drive at.	 */	private double getRangeDrivePower(double startingDistanceFromWall,	                                  double power)	{		double adjustedPower = -((1 - MINIMUM_DRIVE_POWER) - (1 - power)) *				Math.pow(rangeSensor.cmUltrasonic() /								startingDistanceFromWall,						2) + power;		return Range.clip(adjustedPower, MINIMUM_DRIVE_POWER, power);	}	private boolean isWallInRangeThreshold(int threshold)	{		return rangeSensor.cmUltrasonic() <= threshold;	}	void turnButtonPusherRight()	{		if (opModeIsActive())			buttonPusher.setPosition(0);	}	void launchBall(int delay)	{		if (opModeIsActive())		{			liftBallDeployer();		}		sleep(delay);		dropBallDeployer();	}	void liftBallDeployer()	{		if (opModeIsActive())			ballDeployer.setPosition(.75);	}	/**	 * Sleeps until the gyro is done calibrating	 */	void waitForGyroCalibration()	{		telemetry.addData(">", "Calibrating gyro...");		telemetry.update();		sleep(1000);		while (gyroSensor.isCalibrating() && !isStopRequested())		{			idle();			sleep(50);		}		telemetry.addData(">", "Archimedes ready!");		telemetry.update();	}	boolean isDetectingBlueOnRight()	{		return colorSensor.blue() > colorSensor.red();	}	boolean isDetectingRedOnRight()	{		return colorSensor.blue() < colorSensor.red();	}	/**	 * Turns the robot by a set amount of degrees. Slows down as it gets	 * closer to the target heading.	 *	 * @param power       The maximum power that the robot should drive at.	 * @param angleChange The amount of degrees that the robot should turn.	 *	 * @see #getTurnPower(double, double)	 */	void turn(double power, int angleChange)	{		if (opModeIsActive())		{			rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			changeTargetGyroHeading(angleChange);			while (!isHeadingReached(angleChange) && opModeIsActive())			{				double turnPower = getTurnPower(angleChange, power);				leftDriveMotor.setPower(-turnPower);				rightDriveMotor.setPower(turnPower);			}		}		stopDriveMotors();	}	private boolean isHeadingReached(double heading)	{		if (heading > 0)		{			return (getGyroHeading() > targetGyroHeading_);		}		else		{			return (getGyroHeading() < targetGyroHeading_);		}	}	/**	 * Generates a parabolic function that slows down that robot as it gets	 * closer to its target heading when turning.	 *	 * @param angleChange The amount of degrees that the robot it turning.	 * @param power       The maximum power the robot should turn at.	 *	 * @return The power the robot should turn at.	 */	private double getTurnPower(double angleChange, double power)	{		double adjustedPower = -((1 - MINIMUM_TURN_POWER) - (1 - power)) *				Math.pow((1 - Math.abs(targetGyroHeading_ - getGyroHeading())) /						Math.abs(angleChange), 2) + power;		if (angleChange > 0)		{			return Range.clip(adjustedPower, MINIMUM_TURN_POWER, power);		}		else		{			return Range.clip(-adjustedPower, -power, -MINIMUM_TURN_POWER);		}	}	private void changeTargetGyroHeading(int headingChange)	{		targetGyroHeading_ += headingChange;	}	void turnButtonPusherLeft()	{		if (opModeIsActive())			buttonPusher.setPosition(.6);	}	/**	 * Drives for a set amount of time.	 *	 * @param power The power the robot should drive at.	 * @param time  The time in milliseconds that the robot should drive for.	 */	void timeDrive(double power, int time)	{		if (opModeIsActive())		{			leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);			leftDriveMotor.setPower(power);			rightDriveMotor.setPower(power);			double startTime = elapsedTime.milliseconds();			while (startTime + time > elapsedTime.milliseconds() &&					opModeIsActive())			{				sleep(50);				idle();			}		}		stopDriveMotors();	}	boolean isAlignedWithBeacon()	{		// TODO Test this function		final int GYRO_ALIGNMENT_THRESHOLD = 2;		final double WALL_RANGE_THRESHOLD = 4.5;		final double LINE_THRESHOLD = 1.5;		// This assumes that the target targetGyroHeading is directly towards		// one of the walls		boolean isGyroAlignedWithWall =				Math.abs(getGyroHeading()) - targetGyroHeading_ <=						GYRO_ALIGNMENT_THRESHOLD;		boolean isRobotTooCloseToWall = rangeSensor.cmUltrasonic() <=				WALL_RANGE_THRESHOLD;		return isGyroAlignedWithWall &&				isLineDetected(LINE_THRESHOLD) && !isRobotTooCloseToWall;	}}