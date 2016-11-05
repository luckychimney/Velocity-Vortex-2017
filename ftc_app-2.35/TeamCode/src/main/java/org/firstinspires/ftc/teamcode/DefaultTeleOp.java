package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class DefaultTeleOp extends LinearOpMode
{
	private Robot robot = new Robot();

	@Override
	public void runOpMode() throws InterruptedException
	{
		double right_speed;
		double left_speed;
		
		waitForStart();

		while (opModeIsActive())
		{
			if (gamepad1.right_bumper)
			{
				right_speed = gamepad1.right_stick_y * 1;
				left_speed = gamepad1.left_stick_y * 1;
			}
			else if (gamepad1.left_bumper)
			{
				right_speed = gamepad1.right_stick_y * 0.55;
				left_speed = gamepad1.left_stick_y * 0.55;
			}
			else
			{
				right_speed = gamepad1.right_stick_y * 0.75;
				left_speed = gamepad1.left_stick_y * 0.75;
			}

			robot.rightMotor.setPower(right_speed);
			robot.leftMotor.setPower(left_speed);
		}
	}
}
