package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Autonomous 3")
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

			ballLauncher.setPower(0.55);
			drive(DEFAULT_DRIVE_SPEED, 500);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);
			ballLauncher.setPower(0);

			drive(DEFAULT_DRIVE_SPEED, 600);
			turn(0.75, 175);
			timeDrive(-DEFAULT_DRIVE_SPEED, 1000);
		}
	}
}
