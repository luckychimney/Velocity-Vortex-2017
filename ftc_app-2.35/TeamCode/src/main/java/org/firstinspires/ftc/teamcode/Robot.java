package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Robot
{
    DcMotor leftMotor;
    DcMotor rightMotor;

    GyroSensor gyroSensor;

    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    private VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");

    private final int ENCODER_UNITS_PER_REVOLUTION = 1440;
    private final double ENCODER_WHEEL_DIAMETER = 50.8;
    final double ENCODER_UNITS_PER_MILLIMETER = (ENCODER_UNITS_PER_REVOLUTION/(ENCODER_WHEEL_DIAMETER * Math.PI));

    final double GYRO_DRIVE_COEFFICIENT = 0.1;

    public void init(HardwareMap hardwareMap)
    {
        leftMotor = hardwareMap.dcMotor.get("left motor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor = hardwareMap.dcMotor.get("right motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "AVjSL4f/////AAAAGRrqOmjjwUvcra1uL+pn/W8AoLn03Yj7g6Aw+VGRAI+CkzXWFw/7FLW09TYRSzxCcQmlWovvlsq9k4DqqxDr+bnAVhsmk+MNEzKyMBqkwMM6BGjEL6ohtkGbMiE+sYL0aWgZ+ULu6pPJZQboiH/sEcH2jq8o5zAVe3lbP9E34gCELlHAIzgEta7lXabdjC86OixIDZbdEBpE5UTGPRFKTbYgKFVNoouFgUT4hs5MiqD21DwbubgmSe+WOVyi3G4WTkJowT9jx1XlOrUXwc6kfyArQ+DFQNXEghwAXhC9FOEWijQTKSG+TDq7XePfRICqLPEdl4aYUixHn6OCCPZ85o7bEaBYf74ZddKg7IBTCOsg";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        beacons.activate();

        hardwareMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();
    }
}
