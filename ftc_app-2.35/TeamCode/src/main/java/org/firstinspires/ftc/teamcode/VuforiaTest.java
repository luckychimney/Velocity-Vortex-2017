package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Vuforia Test")
@Disabled
public class VuforiaTest extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
				R.id.cameraMonitorViewId);
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
		parameters.vuforiaLicenseKey =
				"AVjSL4f/////AAAAGRrqOmjjwUvcra1uL+pn/W8AoLn03Yj7g6Aw+VGRAI+CkzXWFw/7FLW09TYRSzxCcQmlWovvlsq9k4DqqxDr+bnAVhsmk+MNEzKyMBqkwMM6BGjEL6ohtkGbMiE+sYL0aWgZ+ULu6pPJZQboiH/sEcH2jq8o5zAVe3lbP9E34gCELlHAIzgEta7lXabdjC86OixIDZbdEBpE5UTGPRFKTbYgKFVNoouFgUT4hs5MiqD21DwbubgmSe+WOVyi3G4WTkJowT9jx1XlOrUXwc6kfyArQ+DFQNXEghwAXhC9FOEWijQTKSG+TDq7XePfRICqLPEdl4aYUixHn6OCCPZ85o7bEaBYf74ZddKg7IBTCOsg";
		parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
		Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

		VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
		beacons.get(0).setName("Wheels");
		beacons.get(1).setName("Tools");
		beacons.get(2).setName("Lego");
		beacons.get(3).setName("Gears");

		waitForStart();

		beacons.activate();

		while (opModeIsActive())
		{
			for (VuforiaTrackable beacon : beacons)
			{
				OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener())
						.getPose();

				if (pose != null)
				{
					VectorF translation = pose.getTranslation();

					double distanceFromVuforiaTrackable = translation.get(0);
					double distanceFromWall = translation.get(2);

					double degreesToTurn = Math.abs(Math.toDegrees(
							Math.atan2(distanceFromVuforiaTrackable, distanceFromWall))) - 90;
					double distanceToDrive = Math
							.sqrt(Math.pow(distanceFromVuforiaTrackable, 2) + Math
									.pow(Math.abs(distanceFromWall), 2));

					telemetry.addData("Name", beacon.getName());
					telemetry.addData("X", Math.round(distanceFromVuforiaTrackable) + "mm");
					telemetry.addData("Z", Math.abs(Math.round(distanceFromWall)) + "mm");
					telemetry.addData("Degrees", Math.round(degreesToTurn) + "°");
					telemetry.addData("Distance", Math.round(distanceToDrive) + "mm");
				}
				else
				{
					telemetry.clearAll();
				}
			}
			telemetry.update();
		}
	}
}
