package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Autonomous")
public class DefaultAutonomous extends LinearOpMode
{
    private Robot robot = new Robot();

    private int degreesToTurn;
    private int distanceToDrive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);

        while(robot.gyroSensor.isCalibrating())
        {
            telemetry.addData(">", "Calibrating gyro...");
            telemetry.update();
            idle();
            sleep(50);
        }

        telemetry.addData(">", "Robot ready!");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Robot running...");
        telemetry.update();

        while (!isVuforiaTrackableFound(robot.beacons) && opModeIsActive())
        {
            drive(0.2, 50);
            sleep(1500);
        }

        turn(0.2, degreesToTurn);

        drive(0.2, distanceToDrive);

        turn(0.2, 90 - degreesToTurn);

        drive(0.2, 450);
    }

    private boolean isVuforiaTrackableFound(VuforiaTrackables beacons)
    {
        for (VuforiaTrackable beacon : beacons)
        {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

            if (pose != null)
            {
                VectorF translation = pose.getTranslation();

                double x = translation.get(0);
                double z = translation.get(2);

                degreesToTurn = (int) Math.round(Math.abs(Math.toDegrees(Math.atan2(x, z))) - 90);
                distanceToDrive = (int) Math.round(Math.sqrt(Math.pow(x, 2) + Math.pow(Math.abs(z), 2)));

                return true;
            }
        }
        return false;
    }

    private void stopDriveMotors()
    {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void drive(double power, int distance)
    {
        double leftAdjustedPower;
        double rightAdjustedPower;

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftAdjustedPower = power * (getAbsGyroHeading()*robot.GYRO_DRIVE_COEFFICIENT);
        rightAdjustedPower = -leftAdjustedPower;

        double encoderUnitsToDrive = robot.ENCODER_UNITS_PER_MILLIMETER * distance;

        robot.leftMotor.setPower(leftAdjustedPower);
        robot.rightMotor.setPower(rightAdjustedPower);

        if (distance > 0)
        {
            while (robot.leftMotor.getCurrentPosition() < encoderUnitsToDrive && opModeIsActive())
            {
                idle();
                sleep(50);
            }
        }
        else
        {
            while (robot.leftMotor.getCurrentPosition() > encoderUnitsToDrive && opModeIsActive())
            {
                idle();
                sleep(50);
            }
        }

        stopDriveMotors();
    }

    private int getAbsGyroHeading()
    {
        if (robot.gyroSensor.getHeading() > 180)
        {
            return robot.gyroSensor.getHeading() - 360;
        }
        else
        {
            return robot.gyroSensor.getHeading();
        }
    }

    private boolean headingReached(double heading)
    {
        if (heading > 0)
        {
            return(getAbsGyroHeading() >= heading);
        }
        else
        {
            return(getAbsGyroHeading() <= heading);
        }
    }

    private void turn(double power, int degrees)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.gyroSensor.resetZAxisIntegrator();

        if (degrees > 0)
        {
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(-power);
        }
        else
        {
            robot.leftMotor.setPower(-power);
            robot.rightMotor.setPower(power);
        }

        while(!headingReached(degrees) && opModeIsActive())
        {
            idle();
            sleep(50);
        }

        stopDriveMotors();
    }
}
