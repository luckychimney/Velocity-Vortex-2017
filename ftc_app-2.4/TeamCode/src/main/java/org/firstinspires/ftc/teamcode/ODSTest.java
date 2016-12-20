package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ODSTest extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeRobot();
		waitForGyroCalibration();
		waitForStart();

		drive(.1, 200);

		while (opModeIsActive())
		{
			telemetry.addData("Gyro Adjustment", getGyroPowerAdjustment());
			telemetry.addData("OPD Ajustment", getOpticalDistanceSensorPowerAdjustment());
			telemetry.addData("Right", rightOpticalDistanceSensor.getLightDetected());
			telemetry.addData("Left", leftOpticalDistanceSensor.getLightDetected());
			telemetry.addData("Combined Adjustment", getCombinedPowerAdjustment());
			telemetry.update();
		}
	}
}
