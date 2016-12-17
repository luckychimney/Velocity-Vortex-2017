package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class squareDrive extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initializeRobot();
		waitForGyroCalibration();
		waitForStart();

		for (int i = 0; i < 4; i++)
		{
			drive(DEFAULT_DRIVE_SPEED, 500);
			turn(DEFAULT_TURN_SPEED, 90);
		}
	}
}
