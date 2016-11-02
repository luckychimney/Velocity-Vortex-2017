package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class Robot
{
	final double GYRO_DRIVE_COEFFICIENT = 0.1;
	private final int ENCODER_UNITS_PER_REVOLUTION = 1440;
	private final double ENCODER_WHEEL_DIAMETER = 50.8;
	final double ENCODER_UNITS_PER_MILLIMETER = (ENCODER_UNITS_PER_REVOLUTION / (ENCODER_WHEEL_DIAMETER * Math.PI));

	DcMotor leftMotor;
	DcMotor rightMotor;
	GyroSensor gyroSensor;
	VuforiaTrackables beacons;
	private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

	public void init(HardwareMap hardwareMap) throws FileNotFoundException
	{
		leftMotor = hardwareMap.dcMotor.get("left motor");
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightMotor = hardwareMap.dcMotor.get("right motor");
		rightMotor.setDirection(DcMotor.Direction.FORWARD);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		Scanner fileIn = new Scanner(new File("Vuforia Key.txt"));
		parameters.vuforiaLicenseKey = fileIn.nextLine();
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
		Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

		beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
		beacons.get(0).setName("Wheels");
		beacons.get(1).setName("Tools");
		beacons.get(2).setName("Lego");
		beacons.get(3).setName("Gears");
		beacons.activate();

		gyroSensor = hardwareMap.gyroSensor.get("gyro");
		gyroSensor.calibrate();
	}

	DcMotor getEncoderWheel()
	{
		return rightMotor;
	}
}
