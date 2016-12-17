package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class gyroTest extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeRobot();
		waitForGyroCalibration();
		waitForStart();
		while (opModeIsActive())
		{
			telemetry.addData("Abs Gyro", getAbsGyroHeading());
			telemetry.addData("Gyro", gyroSensor.getHeading());
			telemetry.update();
		}
	}
}
