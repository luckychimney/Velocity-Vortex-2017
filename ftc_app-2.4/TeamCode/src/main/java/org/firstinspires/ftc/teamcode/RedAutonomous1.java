package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Delivers two balls into the center vortex, then presses one of the beacon
 * buttons. Is set up on the default red setup position.
 *
 * @author got robot?
 */
@Autonomous(name = "Red: Beacons", group = "Red")
public class RedAutonomous1 extends Archimedes
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
			drive(DEFAULT_DRIVE_POWER, 300);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			stopBallLauncher();

			// Turn toward the beacon line, drive to it and then turn into it.
			turn(DEFAULT_TURN_POWER, -41);
			driveToLine(DEFAULT_DRIVE_POWER, 1150, 0.15, 75);
			turn(DEFAULT_TURN_POWER, -49);

			// Follow the line up to the beacon.
			turnButtonPusherLeft(); // This is needed to expose the color sensor
			followBeaconLineToWall(DEFAULT_DRIVE_POWER, 9);

			// Detect if the robot is lined up with the beacon, if it is then
			// detect what color is the red one and press it
			if (isAlignedWithBeacon())
			{
				sleep(500);
				if (isDetectingRedOnRight())
				{
					turnButtonPusherRight();
					sleep(500);
				}
				timeDrive(.5, 500);
				drive(DEFAULT_DRIVE_POWER, -70);

				// As a safety feature, check the color of the beacon, if it is
				// blue, wait 5 seconds and press the beacon again.
				while (opModeIsActive())
				{
					turnButtonPusherLeft();
					sleep(1000);

					if (isDetectingBlueOnRight())
					{
						setButtonPusherToNeutral();
						sleep(5000);
						timeDrive(.5, 500);
						drive(DEFAULT_DRIVE_POWER, -70);
					}
					else
					{
						stop();
					}
				}
			}
		}
	}
}
