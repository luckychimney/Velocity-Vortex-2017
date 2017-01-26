package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Delivers two balls into the center vortex, then presses one of the beacon
 * buttons. Is set up on the default red setup position.
 *
 * @author got robot?
 */
@Autonomous(name = "Red: Beacons", group = "Red")
public class RedAutonomous1 extends Archimedes
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeArchimedes();
        waitForGyroCalibration();
        waitForStart();
        startArchimedes();

        if (opModeIsActive())
        {
            // Launch balls into center vortex.
            startBallLauncherAtLowPower();
            drive(DEFAULT_DRIVE_POWER, 300);
            sleep(1500);
            launchBall(1000);
            sleep(1500);
            launchBall(1000);
            stopBallLauncher();

            // Turn toward the beacon line, drive to it and then turn into it.
            turn(DEFAULT_TURN_POWER, -41);
            driveToLine(DEFAULT_DRIVE_POWER, DEFAULT_LINE_THRESHOLD, 1150, 75);
            turn(DEFAULT_TURN_POWER, -49);

            // This is needed to expose the color sensor
            turnButtonPusherLeft();

            while (opModeIsActive())
            {
                // Follow the line up to the beacon.
                followLineToWall(DEFAULT_DRIVE_POWER, 9);

                // Detect if the robot is lined up with the beacon, if it is then
                // detect what color is the red one and press it
                if (isAlignedWithBeacon())
                {
                    sleep(500);
                    if (isDetectingRedOnRight())
                    {
                        turnButtonPusherRight();
                        sleep(500);
                    }
                    timeDrive(.5, 500);
                    drive(DEFAULT_DRIVE_POWER, -70);

                    // As a safety feature, check the color of the beacon, if it is
                    // blue, wait 5 seconds and press the beacon again.
                    turnButtonPusherLeft();
                    sleep(1000);

                    if (isDetectingBlueOnRight())
                    {
                        setButtonPusherToNeutral();
                        sleep(5000);
                        timeDrive(.5, 500);
                        drive(DEFAULT_DRIVE_POWER, -70);
                        stop();
                    }
                }
                else
                {
                    drive(DEFAULT_DRIVE_POWER, -350);
                    findLine(DEFAULT_LINE_THRESHOLD, 3);
                }
            }
        }
    }
}
