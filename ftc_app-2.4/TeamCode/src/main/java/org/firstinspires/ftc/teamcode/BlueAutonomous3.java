package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Autonomous 3", group = "Blue")
public class BlueAutonomous3 extends Archimedes
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
			drive(1, 650);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			stopBallLauncher();

			// Drive towards the center vortex, knock off the cap ball, turn
			// around and park.
			drive(1, 300);
			turn(1, 160);
			timeDrive(-DEFAULT_DRIVE_SPEED, 1000);
		}
	}
}
