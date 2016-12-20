package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Encoder Fly Wheel Test")
public class encoderWheelTest extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeRobot();
		waitForStart();
		while (opModeIsActive())
		{
			ballLauncher.setPower(0.5);
			idle();
			sleep(3000);
			launchBall(1000);
			ballLauncher.setPower(0.6);
			idle();
			sleep(3000);
			launchBall(1000);
			ballLauncher.setPower(0.7);
			idle();
			sleep(3000);
			launchBall(1000);
			ballLauncher.setPower(0.8);
			idle();
			sleep(3000);
			launchBall(1000);
		}
	}
}
