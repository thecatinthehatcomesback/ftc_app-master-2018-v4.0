/**
 PotatoAutonomous.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which stone we are on with gamepad1 and knock the jewel off and
 place a glyph or two into the cryptobox.

 Potato is written for use with the mecanum intake and linear slides
 used for the North Super Regionals.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Autonomous(name="New Autonomous", group="CatAuto")
public class NewAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delaytimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedMission = true;
    private HardwareCatBot.SOCKmission mission = HardwareCatBot.SOCKmission.CENTER;

    private HardwareCatBot.StonePos stonePos = HardwareCatBot.StonePos.Nah;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, true, false);
        robot.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        /** how many potato to get boyfriend?
         * 3 whole potato
         * I thought it was 2 whole potato
         * The recession hit us hard
         **/
        telemetry.update();

        robot.resetEncoders();
        idle();

        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // After init is pushed but before Start we can change the delay using dpad up/down
        delaytimer.reset();

        while (!opModeIsActive() ) {
            if (this.isStopRequested()) {
                return;
            }
            // increase if up is pressed and it's been 0.2 seconds since last push
            if (gamepad1.dpad_up && (delaytimer.seconds() > 0.8)) {
                timeDelay += 1;
                delaytimer.reset();
            }
            // decrease if down is pressed and it's been 0.2 seconds since last push
            if (gamepad1.dpad_down && (delaytimer.seconds() > 0.8)) {
                if (timeDelay > 0) {
                    timeDelay -= 1;
                }
                delaytimer.reset();
            }
            if (((gamepad1.dpad_left) && delaytimer.seconds() > 0.8)) {
                if (isRedMission) {
                    isRedMission = false;
                } else {
                    isRedMission = true;
                }
                delaytimer.reset();
            }

            if (gamepad1.dpad_right && delaytimer.seconds() > 0.8) {
                if (stonePos == HardwareCatBot.StonePos.Audience) {
                    stonePos = HardwareCatBot.StonePos.Nah;
                } else {
                    stonePos = HardwareCatBot.StonePos.Audience;
                }
                delaytimer.reset();
            }

            // LED code...
            if (isRedMission) {
                robot.blinky(HardwareCatBot.LED_LightUpType.RED);
                robot.allianceColor = HardwareCatBot.LED_LightUpType.RED;
            } else {
                robot.blinky(HardwareCatBot.LED_LightUpType.BLUE);
                robot.allianceColor = HardwareCatBot.LED_LightUpType.BLUE;
            }

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedMission) {
                telemetry.addData("Alliance:", "Red");
            } else {
                telemetry.addData("Alliance:", "Blue");
            }
            if (stonePos == HardwareCatBot.StonePos.Audience) {
                telemetry.addData("Stone:", "Audience");
            } else {
                telemetry.addData("Stone:", "Nah");
            }
            telemetry.update();

            //Don't need "waitForStart()" since we've been looping waiting for opmode to be enabled.

        }
        /**
         * Runs after hit start
         * DO STUFF FOR MODE!!!!!!!!!!!
         *
         \*/


    }
}