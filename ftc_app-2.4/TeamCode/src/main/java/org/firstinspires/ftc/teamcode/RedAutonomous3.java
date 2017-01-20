package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous 3", group = "Red")
public class RedAutonomous3 extends Archimedes
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeArchimedes();
		waitForGyroCalibration();
		waitForStart();

		if (opModeIsActive())
		{
			telemetry.addData(">", "Archimedes running...");
			telemetry.update();

			// Launch balls into center vortex.
			startBallLauncherForAutonomous();
			drive(1, 675);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			stopBallLauncher();

			// Drive towards the center vortex, knock off the cap ball, turn
			// around and park.
			drive(1, 600);
			turn(1, -100);
			timeDrive(-DEFAULT_DRIVE_SPEED, 1000);
		}
	}
}
