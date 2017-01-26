package org.firstinspires.ftc.teamcode;import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.ColorSensor;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.DcMotorSimple;import com.qualcomm.robotcore.hardware.GyroSensor;import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;import com.qualcomm.robotcore.hardware.Servo;import com.qualcomm.robotcore.util.ElapsedTime;/** * The Archimedes class contains all the methods that run the robot for the * 2016-2017 robot for the FTC team #5037 got robot? * * @author got robot? * @version 2.2 * @since 2017-01-24 */abstract class Archimedes extends LinearOpMode{    final double DEFAULT_DRIVE_POWER = 1.0;    final double DEFAULT_TURN_POWER = 0.85;    final double DEFAULT_LINE_THRESHOLD = 0.15;    final double MINIMUM_DRIVE_POWER = 0.15;    final double MINIMUM_TURN_POWER = 0.35;    private final int ENCODER_UNITS_PER_REVOLUTION = 1440;    private final double ENCODER_WHEEL_DIAMETER = 50.2;    private final double ENCODER_UNITS_PER_MILLIMETER =            (ENCODER_UNITS_PER_REVOLUTION /                    (ENCODER_WHEEL_DIAMETER * Math.PI));    private int lastRawGyroHeading_ = 0;    private int gyroHeading_ = 0;    private int targetGyroHeading_ = 0;    private ElapsedTime elapsedTime = new ElapsedTime();    private DcMotor leftDriveMotor;    private DcMotor rightDriveMotor;    private DcMotor ballLauncher;    private DcMotor ballCollector;    private DcMotor liftMotor1;    private DcMotor liftMotor2;    private Servo ballDeployer;    private Servo capBallGrabber;    private Servo buttonPusher;    private ColorSensor colorSensor;    private ModernRoboticsI2cRangeSensor rangeSensor;    private OpticalDistanceSensor leftOds;    private OpticalDistanceSensor rightOds;    private GyroSensor gyroSensor;    // Robot Setup    /**     * Initializes all of the motors, servos and sensors. Also beings     * calibrating the gyro sensor.     */    void initializeArchimedes()    {        leftDriveMotor = hardwareMap.dcMotor.get("left motor");        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        rightDriveMotor = hardwareMap.dcMotor.get("right motor");        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        ballLauncher = hardwareMap.dcMotor.get("ball launcher");        ballLauncher.setDirection(DcMotor.Direction.REVERSE);        ballLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        ballLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        ballLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);        ballCollector = hardwareMap.dcMotor.get("ball collector");        ballCollector.setDirection(DcMotor.Direction.REVERSE);        ballCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        liftMotor1 = hardwareMap.dcMotor.get("lift motor 1");        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);        liftMotor2 = hardwareMap.dcMotor.get("lift motor 2");        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);        capBallGrabber = hardwareMap.servo.get("cap ball grabber");        capBallGrabber.setDirection(Servo.Direction.FORWARD);        dropCapBallGrabber();        buttonPusher = hardwareMap.servo.get("button pusher");        buttonPusher.setDirection(Servo.Direction.FORWARD);        setButtonPusherToNeutral();        ballDeployer = hardwareMap.servo.get("ball deployer");        ballDeployer.setDirection(Servo.Direction.FORWARD);        dropBallDeployer();        colorSensor = hardwareMap.colorSensor.get("color sensor");        rangeSensor = hardwareMap                .get(ModernRoboticsI2cRangeSensor.class, "range sensor");        leftOds =                hardwareMap.opticalDistanceSensor.get("left ODS");        rightOds =                hardwareMap.opticalDistanceSensor.get("right ODS");        gyroSensor = hardwareMap.gyroSensor.get("gyro");        gyroSensor.calibrate();    }    /**     * Sleeps until the gyro is done calibrating     */    void waitForGyroCalibration()    {        telemetry.addData(">", "Calibrating gyro...");        telemetry.update();        sleep(1000);        while (gyroSensor.isCalibrating() && !isStopRequested())        {            idle();            sleep(50);        }        telemetry.addData(">", "Archimedes ready!");        telemetry.update();    }    void startArchimedes()    {        if (opModeIsActive())        {            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            telemetry.addData(">", "Archimedes running...");            telemetry.update();        }    }    // Movement    /**     * Tells the robot to drive forward a set distance. Slows down as it gets     * closer to the set distance. Also corrects its self if using the gyro     * sensor if it starts to drift.     *     * @param power    The maximum power the robot will drive.     * @param distance The distance the robot should drive in cm.     *     * @see #getGyroPowerAdjustment()     * @see #getDrivePower(double, double)     */    void drive(double power, int distance)    {        final double TARGET_ENCODER_DISTANCE = ENCODER_UNITS_PER_MILLIMETER *                distance;        double leftMotorPower;        double rightMotorPower;        if (opModeIsActive())        {            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            while (!isDriveDistanceReached(TARGET_ENCODER_DISTANCE) &&                    opModeIsActive())            {                leftMotorPower =                        getDrivePower(power, TARGET_ENCODER_DISTANCE) -                                getGyroPowerAdjustment();                rightMotorPower =                        getDrivePower(power, TARGET_ENCODER_DISTANCE) +                                getGyroPowerAdjustment();                leftDriveMotor.setPower(leftMotorPower);                rightDriveMotor.setPower(rightMotorPower);                idle();                sleep(50);            }        }        stopDriveMotors();    }    /**     * Turns the robot by a set amount of degrees. Slows down as it gets     * closer to the target heading.     *     * @param power       The maximum power that the robot should drive at.     * @param angleChange The amount of degrees that the robot should turn.     *     * @see #getTurnPower(double, double)     */    void turn(double power, int angleChange)    {        if (opModeIsActive())        {            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            changeTargetGyroHeading(angleChange);            while (!isHeadingReached(angleChange) && opModeIsActive())            {                double turnPower = getTurnPower(power, angleChange);                leftDriveMotor.setPower(-turnPower);                rightDriveMotor.setPower(turnPower);            }        }        stopDriveMotors();    }    /**     * Drives the robot until it finds a line and then drives slightly     * farther. Slows down as it gets closer to the point where the line is     * expected. Also uses gyro correction to correct the drive if it drifts.     *     * @param power             The maximum power the robot should drive at     * @param maximumDistance   The maximum distance the robot should drive before     *                          stopping.     * @param lightThreshold    The threshold in which the robot detects a line     *                          and stops.     * @param postDriveDistance The distance the robot should drive after it     *                          finds the line.     *     * @see #getGyroPowerAdjustment()     * @see #getDrivePower(double, double)     */    void driveToLine(double power, double lightThreshold, int maximumDistance,                     int postDriveDistance)    {        final double MAXIMUM_ENCODER_DISTANCE =                ENCODER_UNITS_PER_MILLIMETER * maximumDistance;        final double POST_LINE_DRIVE_DISTANCE =                ENCODER_UNITS_PER_MILLIMETER * postDriveDistance;        final double COMBINED_DISTANCE =                MAXIMUM_ENCODER_DISTANCE + POST_LINE_DRIVE_DISTANCE;        double leftMotorPower;        double rightMotorPower;        if (opModeIsActive())        {            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            while (!isDriveDistanceReached(COMBINED_DISTANCE) &&                    !isLineDetected(lightThreshold) && opModeIsActive())            {                leftMotorPower = getDrivePower(power, COMBINED_DISTANCE) -                        getGyroPowerAdjustment();                rightMotorPower = getDrivePower(power, COMBINED_DISTANCE) +                        getGyroPowerAdjustment();                leftDriveMotor.setPower(leftMotorPower);                rightDriveMotor.setPower(rightMotorPower);                idle();                sleep(50);            }            while (!isDriveDistanceReached(COMBINED_DISTANCE) &&                    opModeIsActive())            {                leftMotorPower = getDrivePower(power, COMBINED_DISTANCE) -                        getGyroPowerAdjustment();                rightMotorPower = getDrivePower(power, COMBINED_DISTANCE) +                        getGyroPowerAdjustment();                leftDriveMotor.setPower(leftMotorPower);                rightDriveMotor.setPower(rightMotorPower);                idle();                sleep(50);            }        }        stopDriveMotors();    }    /**     * Follows the line that is in front of the beacons up to the wall. Slows     * down as it gets closer to the wall.     *     * @param power        The maximum power the robot should drive at.     * @param wallDistance The distance from the wall the robot should     *                     stop at.     *     * @see #getOdsPowerAdjustment()     * @see #getRangeDrivePower(double, double)     */    void followLineToWall(double power, int wallDistance)    {        double leftMotorPower;        double rightMotorPower;        if (opModeIsActive())        {            final double STARTING_WALL_DISTANCE = rangeSensor.cmUltrasonic();            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            while (!isWallWithinRangeThreshold(wallDistance) &&                    opModeIsActive())            {                leftMotorPower =                        getRangeDrivePower(power, STARTING_WALL_DISTANCE) -                                getOdsPowerAdjustment();                rightMotorPower =                        getRangeDrivePower(power, STARTING_WALL_DISTANCE) +                                getOdsPowerAdjustment();                leftDriveMotor.setPower(leftMotorPower);                rightDriveMotor.setPower(rightMotorPower);                idle();                sleep(50);            }        }        stopDriveMotors();    }    /**     * Drives for a set amount of time.     *     * @param power     The power the robot should drive at.     * @param driveTime The time in milliseconds that the robot should drive for.     */    void timeDrive(double power, int driveTime)    {        final double START_TIME = elapsedTime.milliseconds();        if (opModeIsActive())        {            leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            leftDriveMotor.setPower(power);            rightDriveMotor.setPower(power);            while (START_TIME + driveTime > elapsedTime.milliseconds() &&                    opModeIsActive())            {                sleep(50);                idle();            }        }        stopDriveMotors();    }    void findLine(double lightThreshold, double maximumTurnDegrees)    {        if (opModeIsActive())        {            while (!isLineDetected(lightThreshold))            {                rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                if (rightOds.getLightDetected() > leftOds.getLightDetected())                {                    while (!isLineDetected(lightThreshold) &&                            targetGyroHeading_ + maximumTurnDegrees <                                    getGyroHeading() && opModeIsActive())                    {                        leftDriveMotor.setPower(-MINIMUM_TURN_POWER);                        rightDriveMotor.setPower(MINIMUM_TURN_POWER);                        idle();                        sleep(50);                    }                }                else                {                    while (!isLineDetected(lightThreshold) &&                            targetGyroHeading_ - maximumTurnDegrees <                                    getGyroHeading() && opModeIsActive())                    {                        leftDriveMotor.setPower(MINIMUM_TURN_POWER);                        rightDriveMotor.setPower(-MINIMUM_TURN_POWER);                        idle();                        sleep(50);                    }                }            }        }    }    // Power control    /**     * Generates a parabolic function and slows the robot down while driving     * forwards or backwards using the parameters.     *     * @param driveDistance The distance that the robot is attempting to drive.     * @param power         The maximum power the robot should drive at.     *     * @return The power that the robot should drive at.     */    private double getDrivePower(double power, double driveDistance)    {        double adjustedPower = -((1 - MINIMUM_DRIVE_POWER) - (1 - power)) *                Math.pow(Math.abs(getEncoderWheel().getCurrentPosition()) /                        Math.abs(driveDistance), 2) + power;        if (driveDistance > 0)        {            return adjustedPower;        }        else        {            return -adjustedPower;        }    }    /**     * Generates a parabolic function that slows down that robot as it gets     * closer to its target heading when turning.     *     * @param angleChange The amount of degrees that the robot it turning.     * @param power       The maximum power the robot should turn at.     *     * @return The power the robot should turn at.     */    private double getTurnPower(double power, double angleChange)    {        double adjustedPower = -((1 - MINIMUM_TURN_POWER) - (1 - power)) *                Math.pow((1 - Math.abs(targetGyroHeading_ - getGyroHeading())) /                        Math.abs(angleChange), 2) + power;        if (angleChange > 0)        {            return adjustedPower;        }        else        {            return -adjustedPower;        }    }    /**     * Creates a parabolic function using the range sensor so that the robot     * slows down as it gets closer to the wall.     *     * @param startingWallDistance The distance we started at the wall from     * @param power                The maximum power the robot should drive at     *     * @return The power should drive at.     */    private double getRangeDrivePower(double power, double startingWallDistance)    {        return -((1 - MINIMUM_DRIVE_POWER) - (1 - power)) *                Math.pow(rangeSensor.cmUltrasonic() / startingWallDistance,                        2) + power;    }    // Drive Correction    private double getGyroPowerAdjustment()    {        final double GYRO_DRIVE_COEFFICIENT = 0.04;        return (targetGyroHeading_ - getGyroHeading()) * GYRO_DRIVE_COEFFICIENT;    }    private double getOdsPowerAdjustment()    {        final double OPTICAL_DISTANCE_SENSOR_COEFFICIENT = 0.65;        return (rightOds.getLightDetected() - leftOds.getLightDetected()) *                OPTICAL_DISTANCE_SENSOR_COEFFICIENT;    }    // Conditions    /**     * Tells when the distance the robot was driving for was reached.     *     * @param driveDistance The target distance the robot was driving for.     *     * @return True if the distance has been reached.     */    private boolean isDriveDistanceReached(double driveDistance)    {        return Math.abs(getEncoderWheel().getCurrentPosition()) >=                Math.abs(driveDistance);    }    private boolean isHeadingReached(double gyroHeading)    {        if (gyroHeading > 0)        {            return (getGyroHeading() > targetGyroHeading_);        }        else        {            return (getGyroHeading() < targetGyroHeading_);        }    }    /**     * Tells if a light has been detected by the optical distance sensors. In     * almost every case this will be used for detecting the lines in front     * of the beacons.     *     * @param lightThreshold The light threshold that a line has been detected.     *     * @return True if a line has been detected.     */    private boolean isLineDetected(double lightThreshold)    {        return leftOds.getLightDetected() > lightThreshold &&                rightOds.getLightDetected() > lightThreshold;    }    private boolean isWallWithinRangeThreshold(int rangeThreshold)    {        return rangeSensor.cmUltrasonic() <= rangeThreshold;    }    boolean isAlignedWithBeacon()    {        final int GYRO_ALIGNMENT_THRESHOLD = 2;        final double WALL_RANGE_THRESHOLD = 5;        final double LINE_THRESHOLD = 1.5;        // This assumes that the target targetGyroHeading is directly towards        // one of the walls        boolean isGyroAlignedWithWall =                Math.abs(getGyroHeading() - targetGyroHeading_) <=                        GYRO_ALIGNMENT_THRESHOLD;        boolean isRobotTooCloseToWall = rangeSensor.cmUltrasonic() <=                WALL_RANGE_THRESHOLD;        return isGyroAlignedWithWall && !isRobotTooCloseToWall                && isLineDetected(LINE_THRESHOLD);    }    boolean isDetectingBlueOnRight()    {        return colorSensor.blue() > colorSensor.red();    }    boolean isDetectingRedOnRight()    {        return colorSensor.blue() < colorSensor.red();    }    // Actions    /**     * Stops both drive motors and resets the encoder wheel.     */    private void stopDriveMotors()    {        leftDriveMotor.setPower(0.0);        rightDriveMotor.setPower(0.0);        getEncoderWheel().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    }    /**     * Sets the power for both lift motors     *     * @param power The power the lift should run at.     */    void setLiftPower(double power)    {        if (opModeIsActive())        {            liftMotor1.setPower(-power);            liftMotor2.setPower(liftMotor1.getPower());        }    }    void launchBall(int delay)    {        if (opModeIsActive())        {            liftBallDeployer();            sleep(delay);            dropBallDeployer();        }    }    void liftBallDeployer()    {        if (opModeIsActive())            ballDeployer.setPosition(0.75);    }    void dropBallDeployer()    {        if (opModeIsActive())            ballDeployer.setPosition(1.0);    }    void startBallCollector()    {        if (opModeIsActive())            ballCollector.setPower(1.0);    }    void stopBallCollector()    {        if (opModeIsActive())            ballCollector.setPower(0.0);    }    void reverseBallCollector()    {        if (opModeIsActive())            ballCollector.setPower(-1.0);    }    void liftCapBallGrabber()    {        if (opModeIsActive())            capBallGrabber.setPosition(0.85);    }    void clampCapBallGrabber()    {        if (opModeIsActive())            capBallGrabber.setPosition(0.5);    }    void dropCapBallGrabber()    {        if (opModeIsActive())            capBallGrabber.setPosition(0.0);    }    void startBallLauncherAtLowPower()    {        if (opModeIsActive())            ballLauncher.setPower(0.55);    }    void startBallLauncherAtHighPower()    {        if (opModeIsActive())            ballLauncher.setPower(0.75);    }    void stopBallLauncher()    {        ballLauncher.setPower(0.0);    }    void turnButtonPusherLeft()    {        if (opModeIsActive())            buttonPusher.setPosition(0.6);    }    void turnButtonPusherRight()    {        if (opModeIsActive())            buttonPusher.setPosition(0.0);    }    void setButtonPusherToNeutral()    {        if (opModeIsActive())            buttonPusher.setPosition(0.3);    }    // Getters, setters and other    /**     * Calculate the gyro heading by taking the raw gyro heading taken from     * the sensor and converting it into a more usable value.     *     * @return The gyro heading.     */    private int getGyroHeading()    {        int rawGyroHeading = gyroSensor.getHeading();        // 0->360        if (rawGyroHeading - 180 > lastRawGyroHeading_)        {            gyroHeading_ =                    gyroHeading_ + rawGyroHeading - 360 - lastRawGyroHeading_;        }        // 359->0        else if (rawGyroHeading + 180 < lastRawGyroHeading_)        {            gyroHeading_ =                    gyroHeading_ + rawGyroHeading + 360 - lastRawGyroHeading_;        }        else        {            gyroHeading_ =                    gyroHeading_ + rawGyroHeading - lastRawGyroHeading_;        }        lastRawGyroHeading_ = rawGyroHeading;        return gyroHeading_;    }    DcMotor getLeftDriveMotor()    {        return leftDriveMotor;    }    DcMotor getRightDriveMotor()    {        return rightDriveMotor;    }    private DcMotor getEncoderWheel()    {        return rightDriveMotor;    }    private void changeTargetGyroHeading(int headingChange)    {        targetGyroHeading_ += headingChange;    }}