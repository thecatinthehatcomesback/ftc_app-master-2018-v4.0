/*
    NewTeleOp.java

    TeleOp mode for our six wheel drive train.
    Currently not in use since we are using a
    mecanum drive train.

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
    private CatBotHardware.TeleOpDriveMode driveMode = CatBotHardware.TeleOpDriveMode.TankDrive;


    /* Declare OpMode members. */
    CatBotHardware robot; // use the class created for the hardware

    // constructor for class
    public NewTeleOp() {
        robot = new CatBotHardware();
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


        // Go!!!
        runtime.reset();


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
