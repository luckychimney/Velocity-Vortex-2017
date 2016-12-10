package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous")
public class RedAutonomous extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initRobot();

		telemetry.addData(">", "Calibrating gyro...");
		telemetry.update();

		sleep(1000);

		waitForGyroCalibration();

		telemetry.addData(">", "Robot ready!");
		telemetry.update();

		waitForStart();

		if (opModeIsActive())
		{
			telemetry.addData(">", "Robot running...");
			telemetry.update();

			ballLauncher.setPower(0.75);

			drive(1, 304);

			sleep(1500);
			launchBall(1000);
			sleep(1500);
			launchBall(1000);
			sleep(1500);

			ballLauncher.setPower(0);

			drive(1, 700);
			turn(.75, -135);
			timeDrive(-1, 1000);
		}
	}
}
