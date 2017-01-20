package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous 2", group = "Red")
public class RedAutonomous2 extends Archimedes
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeArchimedes();
		waitForGyroCalibration();
		waitForStart();
		startArchimedes();

		if (opModeIsActive())
		{
			// Launch balls into center vortex.
			startBallLauncherForAutonomous();
			drive(1, 300);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			stopBallLauncher();

			// Drive towards the center vortex, knock off the cap ball, turn
			// around and park.
			drive(DEFAULT_DRIVE_SPEED, 600);
			turn(DEFAULT_DRIVE_SPEED, -135);
			timeDrive(-0.65, 1000);
		}
	}
}
