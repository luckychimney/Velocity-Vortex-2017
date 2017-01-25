package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue: Parking", group = "Blue")
public class BlueAutonomous2 extends Archimedes
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
			startBallLauncherAtLowPower();
			drive(DEFAULT_DRIVE_POWER, 300);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			stopBallLauncher();

			// Drive towards the center vortex, knock off the cap ball, turn
			// around and park.
			drive(DEFAULT_DRIVE_POWER, 500);
			turn(DEFAULT_TURN_POWER, 175);
			timeDrive(-0.65, 1000);
		}
	}
}
