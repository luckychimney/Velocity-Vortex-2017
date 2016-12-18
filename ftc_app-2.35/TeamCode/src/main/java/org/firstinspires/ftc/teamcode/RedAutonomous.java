package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous 1")
public class RedAutonomous extends Robot
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

			ballLauncher.setPower(0.45);
			drive(DEFAULT_DRIVE_SPEED, 304);
			launchBall(1500);
			launchBall(1500);
			ballLauncher.setPower(0);
			drive(DEFAULT_DRIVE_SPEED, 700);
			turn(DEFAULT_TURN_SPEED, -135);
			timeDrive(-DEFAULT_DRIVE_SPEED, 1000);
		}
	}
}
