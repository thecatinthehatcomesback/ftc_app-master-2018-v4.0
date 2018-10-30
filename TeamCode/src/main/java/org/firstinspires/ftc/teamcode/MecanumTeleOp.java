/*
    MechanumTeleOp.java

    A Linear opmode class to be our the TeleOp method
    to use for the driver controlled period.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Mechanum TeleOp", group="CatTeleOp")

public class MecanumTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    MecanumHardware robot;  // Use the mecanum class created for the hardware

    // Our constructor for this class
    public MecanumTeleOp() {
        robot = new MecanumHardware();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Go!
        runtime.reset();
        double driveSpeed;
        double leftFront = 0;
        double rightFront = 0;
        double leftBack = 0;
        double rightBack = 0;
        double SF;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            //  ---  SPEED BOOST!!!  --- //
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.4;
            } else {
                driveSpeed = 0.6;
            }
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            leftFront  = -gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            rightFront = -gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            leftBack   = -gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
            rightBack  = -gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;

            /*
            leftFront = robot.limitRange(leftFront, -1.0, 1.0) * driveSpeed;
            rightFront = robot.limitRange(rightFront, -1.0, 1.0) * driveSpeed;
            leftBack = robot.limitRange(leftBack, -1.0, 1.0) * driveSpeed;
            rightBack = robot.limitRange(rightBack, -1.0, 1.0) * driveSpeed;
            */

            SF = robot.findScaleFactor(leftFront, rightFront, leftBack, rightBack);
            leftFront  = leftFront  * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack   = leftBack   * SF * driveSpeed;
            rightBack  = rightBack  * SF * driveSpeed;


            // drive //
            robot.drive(leftFront, rightFront, leftBack, rightBack);


            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Identifier Servo control
            if (gamepad2.dpad_up) {
                robot.markerUp();
            }
            if (gamepad2.dpad_down) {
                robot.markerDown();
            }

            // Hook Controls
            if (gamepad2.dpad_left) {
                robot.hookClamp();
            }
            if (gamepad2.dpad_right) {
                robot.hookRelease();
            }

            robot.tailMotor.setPower(-gamepad2.right_stick_y);

            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            telemetry.addData("tail encoder:",robot.tailMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
