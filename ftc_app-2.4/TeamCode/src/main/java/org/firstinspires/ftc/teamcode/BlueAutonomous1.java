package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Autonomous 1", group = "Blue")
public class BlueAutonomous1 extends Archimedes
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
			drive(DEFAULT_DRIVE_SPEED, 300);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			stopBallLauncher();

			// Go towards the beacon line and turn onto it.
			turn(0.5, 48);
			driveToLine(0.85, 1450, 0.15, 75);
			turn(0.5, 42);

			// Follow the line up to the beacon, detect the color and press
			// the right button.
			turnButtonPusherLeft();
			followBeaconLineToWall(0.2, 300, 10);
			sleep(500);
			if (isDetectingBlueOnRight())
			{
				turnButtonPusherRight();
			}
			sleep(750);
			timeDrive(.5, 500);
			drive(DEFAULT_DRIVE_SPEED, -125);

			// As a safety feature, check to see the color of the beacon, if it
			// is red, wait 5 seconds and press the beacon again.
			while (opModeIsActive())
			{
				turnButtonPusherLeft();
				sleep(1000);

				if (isDetectingRedOnRight())
				{
					setButtonPusherToNeutral();
					sleep(5000);
					timeDrive(.5, 750);
					drive(0.5, -125);
				}
				else
				{
					stop();
				}
			}
		}
	}
}
