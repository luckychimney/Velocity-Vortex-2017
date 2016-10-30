package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class DefaultTeleOp extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            double right_speed;
            double left_speed;

            if (gamepad1.right_bumper) {
                if (gamepad1.right_stick_y >= 0) {
                    right_speed = 1;
                } else {
                    right_speed = -1;
                }

                if (gamepad1.left_stick_y >= 0) {
                    left_speed = 1;
                } else {
                    left_speed = -1;
                }
            } else {
                if (Math.abs(gamepad1.right_stick_y) <= 0.2) {
                    right_speed = 0;
                } else {
                    right_speed = gamepad1.right_stick_y * 0.75;
                }

                if (Math.abs(gamepad1.left_stick_y) <= 0.2) {
                    left_speed = 0;
                } else {
                    left_speed = gamepad1.left_stick_y * 0.75;
                }
            }

            robot.rightMotor.setPower(right_speed);
            robot.leftMotor.setPower(left_speed);
        }
    }
}
