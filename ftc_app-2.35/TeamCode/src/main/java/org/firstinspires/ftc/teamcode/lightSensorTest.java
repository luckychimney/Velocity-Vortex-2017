package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by pjtnt11 on Tue 6.
 */

@Autonomous
public class lightSensorTest extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		AnalogInput lightSensor;

		lightSensor = hardwareMap.analogInput.get("Light sensor");

		waitForStart();

		while(opModeIsActive())
		{
			telemetry.addData("Light", lightSensor.getVoltage());
			telemetry.update();
		}
	}
}
