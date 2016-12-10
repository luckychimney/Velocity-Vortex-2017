package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by pjtnt11 on Sat 10.
 */

@Autonomous(name = "Simple Forward Drive")
public class simpleForwardDrive extends Robot
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		initRobot();
		sleep(1000);
		waitForGyroCalibration();
		waitForStart();
		if(opModeIsActive())
		{
			drive(1, 9999);
		}
	}
}
