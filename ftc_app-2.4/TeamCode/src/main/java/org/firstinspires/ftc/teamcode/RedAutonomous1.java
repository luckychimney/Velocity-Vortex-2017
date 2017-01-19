package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous 1")
public class RedAutonomous1 extends Archimedes
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

			ballLauncher.setPower(0.55);
			drive(1, 304);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			ballLauncher.setPower(0);

			turn(DEFAULT_TURN_SPEED, -41);
			drive(1, 1000);
			driveToLine(0.15, 300, 0.15, 75);
			turn(0.5, -49);
			turnButtonPusherLeft();
			followBeaconLine(0.25, 300, 10);
			sleep(500);
			if(hasDetectedRed())
				turnButtonPusherRight();
			sleep(750);
			timeDrive(.5, 500);
			drive(1, -125);
			turnButtonPusherLeft();
			sleep(500);
			if(hasDetectedBlue())
			{
				setButtonPusherToNeutral();
				sleep(5000);
				timeDrive(.5, 750);
				drive(1, -125);
				stop();
			}
			turn(1, 90);
			drive(DEFAULT_DRIVE_SPEED, 1000);
			driveToLine(0.15, 200, 0.15, 75);

			turn(.5, -90);
			//turnToLine(.25, -90, 0.5);
			turnButtonPusherLeft();
			followBeaconLine(0.25, 300, 10);
 			sleep(500);
			if(hasDetectedRed())
			{
				turnButtonPusherRight();
			}
			sleep(750);
			timeDrive(DEFAULT_DRIVE_SPEED, 550);
			timeDrive(-DEFAULT_DRIVE_SPEED, 500);
		}
	}
}
