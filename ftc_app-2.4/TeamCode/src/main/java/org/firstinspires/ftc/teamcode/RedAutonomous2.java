package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous 2 ")
public class RedAutonomous2 extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeRobot();
		waitForGyroCalibration();
		waitForStart();

		if (opModeIsActive())
		{
			telemetry.addData(">", "Robot running...");
			telemetry.update();

			ballLauncher.setPower(0.75);

			drive(DEFAULT_DRIVE_SPEED, 304);

			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);

			ballLauncher.setPower(0);

			drive(DEFAULT_DRIVE_SPEED, 900);

		}
	}
}
