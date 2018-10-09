/**
 MecTesterAutonomous.java

 A Linear OpMode class to be place to test code
 both old and new.  We constantly edit this, taking
 out and adding in code.  This is never the same.


 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Mec Testing Autonomous", group="CatAuto")
public class MecTesterAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    MecanumHardware robot = new MecanumHardware();   // Use the mecanum hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Initialize the drive system variables.  The init() method of
         * our hardware class does all the work instead of copying every
         * new programming...
         */
        robot.init(hardwareMap, this);
        robot.IMUinit();

        // Send telemetry message to signify robot is waiting
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();

        robot.resetEncoders();
        idle();
        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :7d  :7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition(),
                robot.leftBackMotor.getCurrentPosition(),
                robot.rightBackMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        /**
         * Runs after hit start
         * DO STUFF FOR MODE!!!!!!!!!!!
         *
         \*/

        // Drive forward, back, left, right...
        robot.mecTurn(MecanumHardware.TURN_SPEED, 90, 3);
        robot.robotWait(2.0);
        robot.mecTurn(MecanumHardware.TURN_SPEED, -90, 3);
        robot.robotWait(2.0);


        /**
         * What we plan to test in this autonomous:
         *
         * Test the new mecDriveVertical...  Make sure it is working as planned             <3
         * Test the new mecDriveHorizontal...  Make sure it is working as planned           <3
         * Test the new mecTurn...  Get the angle + and - correct orientations              <3
         */
    }
}