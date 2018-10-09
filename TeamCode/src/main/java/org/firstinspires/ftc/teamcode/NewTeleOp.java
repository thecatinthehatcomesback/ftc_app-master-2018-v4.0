/*
    GruviaTeleOp.java

    A Linear opmode class to be our main teleop method
    This was used with the gripper for the 2 qualifying tournaments
     It has now been replaced when we went to the mecanum intake.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name="New TeleOp", group="CatTeleOp")

public class NewTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime driveModeSwitch     = new ElapsedTime();
    private HardwareCatBot.TeleOpDriveMode driveMode = HardwareCatBot.TeleOpDriveMode.TankDrive;


    /* Declare OpMode members. */
    HardwareCatBot robot; // use the class created for the hardware

    // constructor for class
    public NewTeleOp() {
        robot = new HardwareCatBot();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize/INIT the hardware
        robot.init(hardwareMap, this, false, true);
        robot.gripperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // go
        runtime.reset();
        double driveSpeed;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            /**
             * ---   __________________   ---
             * ---   Gamepad 1 STUFF!!!   ---
             * ---    \/ \/ \/ \/ \/      ---
             */



            /**
             * ---   __________________   ---
             * ---   Gamepad 2 STUFF!!!   ---
             * ---    \/ \/ \/ \/ \/      ---
             */


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */


        }
    }
}
