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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Mec Autonomous", group="CatAuto")
public class MecAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    MecanumHardware robot = new MecanumHardware();      // Use our mecanum hardware
    CatVisionHardware eyes = new CatVisionHardware();   // Doge and vision init
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isCraterSide = true;
    private boolean isParkRedCrater = true;

    private CatVisionHardware.samplingPos samplingPos = CatVisionHardware.samplingPos.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */
        robot.init(hardwareMap, this);
        // Init IMU sensor later when the match starts
        eyes.initDogeforia(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :%7d  :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.leftBackMotor.getCurrentPosition(),
                robot.rightBackMotor.getCurrentPosition());
        telemetry.update();

        robot.markerUp();
        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() ) {
            if (this.isStopRequested()) {
                return;
            }
            // Increases the amount of time we wait
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                timeDelay += 1;
                delayTimer.reset();
            }
            // Decreases the amount of time we wait
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                if (timeDelay > 0) {
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            // Changes which alliance color we are
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                if (isRedAlliance) {
                    isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                }
                delayTimer.reset();
            }

            //change which crater we'll park in :)
            if (((gamepad1.a) && delayTimer.seconds() > 0.8)) {
                if (isParkRedCrater) {
                    isParkRedCrater = false;
                } else {
                    isParkRedCrater = true;
                }
                delayTimer.reset();
            }

            // Changes which side of the lander our robot starts at
            if (((gamepad1.dpad_right) && delayTimer.seconds() > 0.8)) {
                if (isCraterSide) {
                    isCraterSide = false;
                } else {
                    isCraterSide = true;
                }
                delayTimer.reset();
            }

            //// TODO: 9/21/2018 Eventually get LED code working...
            // LED code...
            if (isRedAlliance) {
               // robot.blinky(HardwareCatBot.LED_LightUpType.RED);
               // robot.allianceColor = HardwareCatBot.LED_LightUpType.RED;
            } else {
               // robot.blinky(HardwareCatBot.LED_LightUpType.BLUE);
              //  robot.allianceColor = HardwareCatBot.LED_LightUpType.BLUE;
            }

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            if (isCraterSide) {
                telemetry.addData("Lander Side: ", "Crater");
            } else {
                telemetry.addData("Lander Side: ", "Depot");
            }

            if (isParkRedCrater) {
                telemetry.addData("Parking Crater: ", "Red");
            } else {
                telemetry.addData("Parking Crater: ", "Blue");
            }
            telemetry.update();

            /**
             * We don't need to "waitForStart()" here since we've been
             * looping all this time and waiting for opMode to be enabled.
             */

        }

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */
        robot.IMUinit();

        /**
         * Runs after hit start
         * DO STUFF FOR OPMODE!!!!!!!!!!!
         *
         \*/


        // Lower robot here
        //robot.lowerRobot();
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,3.0,2.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED,3.0,2.0);
        robot.mecDriveHorizontal(robot.DRIVE_SPEED,-3.0,2.0);

        // Find and store the values of the sampling
       samplingPos = eyes.findGoldPos();

        //Delay the amount we selected
        robot.robotWait(timeDelay);

        // Enter the rest of the autonomous based on which side we selected near beginning
        if (isCraterSide) {
            driveCrater();
        }else {
            driveDepot();
        }


        /**
         * What we plan to do in this autonomous:
         *
         */
    }
    public void driveCrater()  throws InterruptedException {
        // Slide if left or right
        robot.mecDriveVertical(MecanumHardware.DRIVE_SPEED, 10, 3.0);
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(MecanumHardware.DRIVE_SPEED, 14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(MecanumHardware.DRIVE_SPEED, -20, 4.0);
                break;
        }
        // Drive forward
        robot.mecDriveVertical(MecanumHardware.DRIVE_SPEED, 8, 4.0);
        robot.mecDriveVertical(MecanumHardware.DRIVE_SPEED, -8, 4.0);
        // Switch back to the center
        switch (samplingPos) {
            case LEFT:
                robot.mecDriveHorizontal(MecanumHardware.DRIVE_SPEED, -14, 4.0);
                break;
            case RIGHT:
                robot.mecDriveHorizontal(MecanumHardware.DRIVE_SPEED, 20, 4.0);
                break;
        }
        robot.mecTurn(robot.TURN_SPEED, -75, 3.0);

        // Drive Forward about 4 foot (To wall)
        robot.mecDriveVertical(robot.DRIVE_SPEED, 44.0, 3.0);
        robot.mecTurn(robot.TURN_SPEED, -135, 3.0);
        robot.mecDriveVertical(robot.DRIVE_SPEED, 38, 3.0);
        robot.markerDown();
        sleep (1000);
        robot.markerUp();
    }
    public void driveDepot() throws InterruptedException {

        // Drive 4 foot and drop mineral off
        robot.mecDriveVertical(robot.DRIVE_SPEED, 58.0, 6.0);
        // Drop Marker
        robot.markerDown();
        sleep(1000);
        robot.markerUp();
        // Turn 45 towards the right crater
        if ((isParkRedCrater && isRedAlliance) || (!isRedAlliance && !isParkRedCrater)) {
            robot.mecTurn(robot.TURN_SPEED, -41, 3.0);
        } else{
            robot.mecTurn(robot.TURN_SPEED, 41, 3.0);
        }

        // Drive Backwards 6 feet (To crater)
        robot.mecDriveVertical(robot.DRIVE_SPEED, -85.0, 8.0);
    }
}
