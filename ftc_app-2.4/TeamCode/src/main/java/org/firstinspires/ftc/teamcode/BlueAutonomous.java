package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Autonomous 1")
public class BlueAutonomous extends Robot
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

			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);

			ballLauncher.setPower(0);

			drive(DEFAULT_DRIVE_SPEED, 700);
			turn(DEFAULT_TURN_SPEED, 175);
			timeDrive(-DEFAULT_DRIVE_SPEED, 1000);
		}
	}
}
