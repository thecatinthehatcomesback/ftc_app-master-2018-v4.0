/*
      HardwareCatBot.java

        An "hardware" class intended to contain common code for accessing the hardware

        This file is a HEAVILY modified version from the FTC SDK.

        Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the Cat in the Hat robot for 2017-2018
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * others tbd.....
 */
public class HardwareCatBot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor           = null;
    public DcMotor  rightMotor          = null;
    public DcMotor  lifterMotor         = null;
    public DcMotor  gripperMotor        = null;
    public DcMotor  intakeMotorRight    = null;
    public DcMotor  intakeMotorLeft     = null;
    public DcMotor  LEDblue             = null;
    public DcMotor  LEDred              = null;
    public DcMotor  relicOut            = null;


    //ModernRoboticsI2cRangeSensor andPeggy;

    public Servo    jewelArm            = null;
    public Servo    jewelFlipper        = null;
    public Servo    intakeRotateyThing  = null;
    public CRServo  relicIn             = null;
    public CRServo  relicElbow          = null;
    public Servo    relicGripper        = null;
    ColorSensor     jewelColors         = null;
    //ColorSensor     TopGlyphCensor      = null;
    public DistanceSensor  topGlyphDist        = null;
    public DistanceSensor  bottGlyphDist        = null;

    // Vuforia
    VuforiaLocalizer    vuforia;
    VuforiaTrackables   relicTrackables;
    RelicRecoveryVuMark vuMark;
    VuforiaTrackable    relicTemplate;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: ANDYMARK Motor Encoder     (Check actual count for next year...)
    static final double     DRIVE_GEAR_REDUCTION    = 16.0/24.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     HYPER_SPEED             = 0.8;
    static final double     CHILL_SPEED             = 0.3;
    static final double     CREEP_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.6;

    static final double     ARM_UP                  = 0.56;
    static final double     ARM_ALMOST_UP           = ARM_UP * 3/4;
    static final double     ARM_DOWN                = 0.0;

    static final double     FLIPPER_LEFT            = 0.90;
    static final double     FLIPPER_CENTER          = 0.35;
    static final double     FLIPPER_RIGHT           = 0.00;

    static final double     SERVO_NEUTRAL_POWER     = 0.5;

    static final double     JEWEL_UP                = 0.9;
    static final double     JEWEL_DOWN              = 0.35;

    static final double     RELIC_GRIPPER_OPEN      = 0.95;
    static final double     RELIC_GRIPPER_GRAB      = 0.25;
    static final double     RELIC_GRIPPER_CLOSE     = 0.05;

    static final double     LEDpower                = 1.0;
    static LED_LightUpType  allianceColor           = LED_LightUpType.RED;

    static final double     TAIL_SWITCH_TIME_MS     = 750;

    enum DRIVE_MODE {
        driveStraight,
        driveOffBalance,
        driveForGlyph //just like drive for straight but also checks for distance of the glyph
    }
    enum TURN_MODE {
        PIVOT,
        TANK
    }
    enum STOP_TYPE {
        BREAK,
        COAST
    }
    enum LED_LightUpType {
        RED,
        BLUE,
        BOTH,
        NONE,
        ALTERNATE
    }

    /**
     * ---   ______________________   ---
     * ---   Enums for our missions   ---
     * ---    \/ \/ \/ \/ \/ \/ \/    ---
     */
    enum SOCKmission {
        RIGHT,
        CENTER,
        LEFT
    }
    enum StonePos {
        Nah,
        Audience
    }
    // enum for TeleOp Driving mode
    enum TeleOpDriveMode {
        TankDrive,
        SingleStick
    }


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;
    private ElapsedTime period  = new ElapsedTime();
    private int baseClear;

    // Stuff
    public ElapsedTime endgameOfAuto = new ElapsedTime();
    public ElapsedTime blinkyTimer = new ElapsedTime();
    public int numTimes = 0;

    /* Constructor */
    public HardwareCatBot(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode, boolean vuforiaEnabled, boolean gripperInit)  throws InterruptedException  {
        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        // Define and Initialize Motors //
        leftMotor   = hwMap.dcMotor.get("left_motor");
        rightMotor  = hwMap.dcMotor.get("right_motor");
        lifterMotor = hwMap.dcMotor.get("lifter_motor");
        relicOut    = hwMap.dcMotor.get("relic_of_the_out");
        if (gripperInit) {  // init gripper if we in Gruvia
            gripperMotor = hwMap.dcMotor.get("gripper_motor");
            gripperMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
            gripperMotor.setPower(0);
            gripperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Reset the gripperMotor when we init
            gripperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);           // Set gripperMotor to RUN_TO_POSITION
            lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Reset the liftMotor when we init
            lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);           // Set lifterMotor to RUN_TO_POSITION
        } else {
            intakeMotorRight = hwMap.dcMotor.get("right_intake_motor");
            intakeMotorLeft  = hwMap.dcMotor.get("left_intake_motor");
            intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
            intakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
            intakeMotorRight.setPower(0);
            intakeMotorLeft.setPower(0);
            lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Reset the liftMotor when we init
            lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);      // Set lifterMotor to RUN_TO_POSITION
        }
        LEDblue             = hwMap.dcMotor.get("led_blue");
        LEDred              = hwMap.dcMotor.get("led_red");
        jewelArm            = hwMap.servo.get("quality_jewel_smackage");
        jewelFlipper        = hwMap.servo.get("quality_arm_idealness");
        intakeRotateyThing  = hwMap.servo.get("intake_rotatey_thing");
        relicElbow          = hwMap.crservo.get("elbow_of_the_relic_arm");
        relicIn             = hwMap.crservo.get("arm_of_the_in");
        relicGripper        = hwMap.servo.get("when_the_arm_grips_and_its_cool");
        jewelColors         = hwMap.colorSensor.get("seeing_red_makes_u_blue");
        //TopGlyphCensor = hwMap.colorSensor.get("top_glyph_sensor");
        topGlyphDist        = hwMap.get(DistanceSensor.class, "top_glyph_sensor");
        bottGlyphDist       = hwMap.get(DistanceSensor.class, "bott_glyph_sensor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);              // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);             // Set to FORWARD if using AndyMark motors
        lifterMotor.setDirection(DcMotor.Direction.FORWARD);            // Set to FORWARD if using AndyMark motors
        relicOut.setDirection(DcMotor.Direction.FORWARD);

        relicIn.setDirection(CRServo.Direction.FORWARD);
        relicElbow.setDirection(CRServo.Direction.REVERSE);


        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);         // Set leftMotor to RUN_WITHOUT_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        // Set rightMotor to RUN_WITHOUT_ENCODER
        relicOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power //
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        lifterMotor.setPower(0);
        LEDblue.setPower(0);
        LEDred.setPower(0);
        relicIn.setPower(0);
        relicOut.setPower(0);
        relicElbow.setPower(0);

        // Set all continuous rotation servos to zero power //
        intakeRotateyThing.setPosition(SERVO_NEUTRAL_POWER);
        relicGripper.setPosition(RELIC_GRIPPER_CLOSE);


        // Set all motors to run without encoders.
        runNoEncoders();

        if (vuforiaEnabled) {

            // init distance sensor
        /*andPeggy = hwMap.get(ModernRoboticsI2cRangeSensor.class, "And_Peggy");*/

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters VUParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View, to save power
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
            VUParameters.vuforiaLicenseKey = "AQUTfjb/////AAAAGZz+VmPB6kQKmJ7YRiW586hGnfqtQHAO7oXSk92nmzrn/8O4MDXAxLMbj6Kc4GJtfvkqObjVJmU39B1wUJM6mWNXG//JcmOmWxaP4AAG163DJfOcQdOV9TOhHsACKW/42t8tipoNjIaBPEdwNhjYZp+2eNTPePDwG43xvLB5I5UEfkOfMV2urrGqCy8gVJr3S1L8XQMgZ6zWSfvQWRubVzBqupxbB7MYl5j49dJUtubvohDdwdfnd7r+8TF5LdqqW4/KBhjLNxMaUQizgEZZ0L91BAivxwfJCDGfRNEZrF0zal696or+rQWQnDdndCBf9Lz9e0W+vftbIHPr4M1JmAakLFoYP9clIMQt3kRqLwNl";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
            VUParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(VUParameters);

            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        }

    }
    /* public void initDogeCV(){

         //  Init the OpenCV...

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.rotateMat = false;

        cryptoboxDetector.enable();
    }
    */



    /**
     * ---   _______________   ---
     * ---   Driving Methods   ---
     * ---    \/ \/ \/ \/ \/   ---
     */
    public void resetEncoders(){

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runNoEncoders(){

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runToPosition(){

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void drive(double left, double right) {

        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }
    /*
    *  Method to perform a relative move, based on encoder counts.
    *  Encoders are not reset as the move is based on the current position.
    *  Move will stop if any of five conditions occur:
    *  1) Move gets to the desired position
    *  2) Move runs out of time
    *  3) Driver stops the opmode running.
    *  4) Distance Sensor gets to a certain range or less
    *  5) Finds line
    */
    public double encoderDrive(double speed,
                             double distance,
                             double timeoutS, DRIVE_MODE driveMode) throws InterruptedException {
        return encoderDrive(speed, distance, timeoutS, driveMode, 0, 0);
    }

    public double encoderDrive(double speed,
                         double distance,
                         double timeoutS, DRIVE_MODE driveMode, int rangeCM, double desiredAngle) throws InterruptedException {
        double distanceDriven = 0;

        int newLeftTarget;
        int newRightTarget;
        ElapsedTime     runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            resetEncoders();

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int)(distance * COUNTS_PER_INCH);
            newRightTarget = (int)(distance * COUNTS_PER_INCH);

            runToPosition();
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Reset the timeout time and start motion.
            runtime.reset();
            if (distance < 0) {
                speed = -speed;
            }

            drive(speed, speed);

            double leftSpeed = speed;
            double rightSpeed = speed;


            boolean keepDriving = true;
            int prevColor = 0;
            int loopCount = 0;

            runtime.reset();
            // List of previous gyro values
            List<Float> samplePoints = new ArrayList<Float>();
            boolean firstPhase = true;

            //DbgLog.msg("CatEncodeDrive Target %d %d Start %d  %d", newLeftTarget, newRightTarget, leftrearMotor.getCurrentPosition(),rightrearMotor.getCurrentPosition()) ;

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    keepDriving) {

                int leftPosition = leftMotor.getCurrentPosition();
                int rightPosition = rightMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftMotor.isBusy() || !rightMotor.isBusy()) {
                    keepDriving = false;
                }

                if ((driveMode == DRIVE_MODE.driveStraight) || (driveMode == DRIVE_MODE.driveForGlyph)) {
                    if ((leftPosition > rightPosition) && ((leftPosition - rightPosition) > 20)) {
                        rightSpeed = rightSpeed * 1.001;
                        drive(leftSpeed, rightSpeed);
                        //DbgLog.msg("CatEncodeDrive Speed %.3f  %.3f +right", leftSpeed, rightSpeed) ;
                    }
                    if ((rightPosition > leftPosition) && ((rightPosition - leftPosition) > 20)) {
                        leftSpeed = leftSpeed * 1.001;
                        drive(leftSpeed, rightSpeed);
                        //DbgLog.msg("CatEncodeDrive Speed %.3f  %.3f +left", leftSpeed, rightSpeed) ;
                    }

                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    if (driveMode == DRIVE_MODE.driveForGlyph) {
                        if (topGlyphDist.getDistance(DistanceUnit.MM) <= 15) {
                            keepDriving = false;
                        }
                    }

                    //failsafeee
                    if (angles.thirdAngle < 60) {
                        keepDriving = false;
                    }
                } else if (driveMode == DRIVE_MODE.driveOffBalance) {

                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    // Just drive straight...
                    drive(leftSpeed, rightSpeed);

                    //// CHANGED THIS FROM 88 to 90 because the imu changed today...
                    // Just cut the crap... STOP AT AN ABSOLUTE!!!
                    if (angles.thirdAngle <= 91) {
                        keepDriving = false;
                    }

                }

                    /*
                        Thought Process
                        1.  Instead of stopping at 0 degrees, use a compare style
                        2.  Drive forward and give a true boolean when tilted forward
                        3.  After a true boolean, start driving until the tilt angle is = to itself for A#OfSeconds

                        In theory, robot should stop in same place every time...
                    */

                    /*
                        Notes:
                        Tilt = Axis is the shafts of the wheels on drive train
                        Using gyro test pitch at 90 is a level tilt for bot
                        Tilt <90 would be backwards
                        Tilt >90 would be forwards
                    */
                // Display it for the driver
                Log.d("catbot", String.format("encoderDrive targ[%5d,%5d], curr[%5d,%5d] power [%.3f,%.3f]",
                        newLeftTarget,  newRightTarget, leftPosition, rightPosition, leftSpeed, rightSpeed));

                opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d :%7d", leftPosition, rightPosition);
                opMode.telemetry.addData("Power", "left %.3f right %.3f", leftSpeed, rightSpeed);
                //opMode.telemetry.addData ("color", "alpha = %d", crgb[0]);
                if(runtime.seconds() > 0) {
                    opMode.telemetry.addData("sample1", "Hz = %.1f", loopCount / runtime.seconds());
                }
                opMode.telemetry.addData("Time","%.4f seconds",runtime.seconds());
                opMode.telemetry.update();
            }

            // Stop all motion;
            drive(0, 0);
            distanceDriven = (double) (leftMotor.getCurrentPosition() / COUNTS_PER_INCH);
            runNoEncoders();
        }
        return distanceDriven;
    }
    /*
   *  Method to turn based on the value of the gyro.
   *  Turn will stop if any of three conditions occur:
   *  1) Turn gets to the desired angle
   *  2) Turn runs out of time
   *  3) Driver stops the opmode running.
   */


    public void gyroturn(double speed,
                         int degrees,
                         double timeoutS, LinearOpMode theOpMode) throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (theOpMode.opModeIsActive()) {
            float targetanglez = getAngle() +degrees;
            absoluteGyro (speed, (int)targetanglez, timeoutS, TURN_MODE.PIVOT);
        }
    }

    public int getAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }
    /*
   *  Method to turn based on the value of the gyro.
   *  Turn will stop if any of three conditions occur:
   *  1) Turn gets to the desired absolute angle
   *  2) Turn runs out of time
   *  3) Driver stops the opmode running.
   *  turns untill it reaches angle based relitive to the starting position
   */
    public void absoluteGyro(double speed,
                             int degrees,
                             double timeoutS, TURN_MODE turnType) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        int prevGyro = 0;
        int gyroLoopCount = 0;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            int targetanglez;
            targetanglez  = degrees;
            boolean leftTurn = (getAngle() > degrees);

            // Turn On RUN_TO_POSITION
            runNoEncoders();

            // reset the timeout time and start motion.
            runtime.reset();
            if (leftTurn) {
                if (turnType == TURN_MODE.PIVOT) {
                    rightMotor.setPower(-speed);
                } else {
                    rightMotor.setPower(-speed / 10.0);
                }
                leftMotor.setPower(speed);

            } else {
                if (turnType == TURN_MODE.PIVOT) {
                    leftMotor.setPower(-speed);
                } else {
                    leftMotor.setPower(-speed / 10);
                }
                rightMotor.setPower(speed);

            }
            // keep looping while we are still active, and there is time left, and both motors are running.
            int wrapAdjust = 0;
            boolean lastWasNegative = getAngle() < 0;
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                int zVal = getAngle();

                /*if ((prevGyro < 0) && (zVal > 0) && (leftTurn)) {
                    wrapAdjust += 360;
                }
                if ((prevGyro > 0) && (zVal < 0) && (!leftTurn)) {
                    wrapAdjust -= 360;
                }*/
                if (prevGyro != zVal) {

                    prevGyro = zVal;
                    gyroLoopCount ++;
                }
                zVal = zVal + wrapAdjust;
                if ((zVal >= targetanglez) && (!leftTurn)){
                    break;
                }
                if ((zVal <= targetanglez) && (leftTurn)) {
                    break;
                }
                Log.d("catbot", String.format("target %d, current %d", targetanglez, zVal));
                opMode.telemetry.addData("Path1",  "Running to %4d", targetanglez);
                opMode.telemetry.addData("Path2", "Current angle is %4d" ,zVal);
                opMode.telemetry.update();

                // Allow time for other processes to run.
                opMode.idle();
            }


            // GYRO Telemetry
            if(runtime.seconds() > 0) {
                opMode.telemetry.addData("sample1", "Hz = %.1f", gyroLoopCount / runtime.seconds());
                //DbgLog.msg("GYRO Hz = %.1f  Turn Rate: %.1f", gyroLoopCount / runtime.seconds(), degrees / runtime.seconds());
            }
            opMode.telemetry.update();

            // Stop all motion;
            drive(0, 0);

            // Turn off RUN_TO_POSITION
            runNoEncoders();
        }
    }



    /**
     * ---   ___________   ---
     * ---   IMU Methods   ---
     * ---   \/ \/ \/ \/   ---
     */
    public void IMUinit () {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

    }

    /**
     * ---   ______________   ---
     * ---   Common Methods   ---
     * ---   \/ \/ \/ \/ \/   ---
     */
    public void robotWait(double seconds) {
        ElapsedTime delaytimer = new ElapsedTime();
        while (opMode.opModeIsActive()  &&  (delaytimer.seconds() < seconds)){
            periodicTeleOpTask();
            //adjustGripper();
            opMode.idle();
            }
    }

    /*
    periodicTeleOpTask - call periodically and the motor will slowly move toward the "target"
        This replaces setting the servo directly - causes the motor to move a bit slower
        which we hope will keep the gears from stripping when it moves between armPositions.
     */
    public void periodicTeleOpTask()  {
        // adjustArm code...
        /*if (Math.abs(lifterMotor.getCurrentPosition() - lifterMotor.getTargetPosition()) > 7) {
            lifterMotor.setPower(0.35);
        } else {
            lifterMotor.setPower(0.2);
        }*/

        /*if (Math.abs(lifterMotor.getCurrentPosition() - lifterMotor.getTargetPosition()) > 7) {
            lifterMotor.setPower(0.35);
        } else {
            if (armIndex == 0) {
                lifterMotor.setPower(0.0);
            }
        }*/

        // After the blinkLength turn the LEDs off...

        // Code for blinky
        if (endgameOfAuto.seconds() > 110 && numTimes < 27) {

            if (blinkyTimer.milliseconds() > 300) {
                // Alternate lights...
                blinky(LED_LightUpType.ALTERNATE);
                // turn off the lights and reset the timer...
                blinkyTimer.reset();
                numTimes++;
            }

            /**
             *  -----OLD CODE-----
             */
            /*if (blinkyTimer.milliseconds() > 500) { // if time greater than .5 sec...
                // turn on lights and reset timer...
                blinky(LED_LightUpType.NONE);
                if (blinkyTimer.milliseconds() > 1000) { // then if time is greater than 1 sec...
                    // turn off the lights and reset the timer...
                    blinkyTimer.reset();
                    numTimes++;
                }
            } else {
                // Turn on LED
                blinky(LED_LightUpType.BOTH);
            }*/
        }
    }


    /**
     * ---   __________________   ---
     * ---   LED light Methods    ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
    public void LEDlights(boolean blueOn, boolean redOn) {
        /* yo turn on LED lights!!! */

        if (blueOn) { // Turn on the Blue Lights...
            LEDblue.setPower(LEDpower);
        } else { // Turn off the Blue Lights...
            LEDblue.setPower(0);
        }
        if (redOn) { // Turn on the Red Lights...
            LEDred.setPower(LEDpower);
        } else { // Turn off the Blue Lights...
            LEDred.setPower(0);
        }
    }
    private boolean LEDisBLUE = false;
    public void blinky(LED_LightUpType type) {
        /* yo turn on LED lights and make them BLINK! */
        // Blink the lights!
        if (type == LED_LightUpType.BLUE) {
            LEDlights(true, false);
        } else if (type == LED_LightUpType.RED) {
            LEDlights(false, true);
        } else if (type == LED_LightUpType.BOTH) {
            LEDlights(true, true);
        } else if (type == LED_LightUpType.NONE) {
            LEDlights(false, false);
        } else if (type == LED_LightUpType.ALTERNATE) {
            /**
             *  Alternate between red and blue LEDs...
             */
            if (LEDisBLUE) {
                LEDlights(false, true);
                LEDisBLUE = false;
            } else {
                LEDlights(true, false);
                LEDisBLUE = true;
            }
        }
    }


    /**
     * ---   _____________   ---
     * ---   Color Methods   ---
     * ---    \/ \/ \/ \/    ---
     */
    public boolean isRed() {
        //crgb
        Log.d("catbot", String.format("CatIsRed r: %d g: %d b: %d", jewelColors.red(), jewelColors.green(),jewelColors.blue()));
        if (jewelColors.red() > jewelColors.blue()) {
            return true;
        } else {
            return false;
        }
    }
    public int findGlyphColor(ColorSensor colorSensor,DistanceSensor distanceSensor) {
        int glyphColor = 0;
        double  avgColor = 0;

        if (distanceSensor.getDistance(DistanceUnit.CM) < 6) {
            avgColor = (colorSensor.red() + colorSensor.blue() + colorSensor.green()) / 3;

            if (avgColor < 80) {
                glyphColor = 1;
            } else if (avgColor > 100){
                glyphColor = 2;
            }
        }

        return glyphColor;
    }


    /**
     * ---   ____________   ---
     * ---   Vumark stuff   ---
     * ---   \/ \/ \/ \/    ---
     */
    public String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public SOCKmission findVuMarks(double timeOut) {
        ElapsedTime outTimer = new ElapsedTime();
        relicTrackables.activate();

        while (outTimer.seconds() < timeOut) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                //telemetry.addData("VuMark", "%s visible", vuMark);
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                //opMode.telemetry.addData("VuMark", "--%s--", String.format("%s",vuMark));
                //opMode.telemetry.update();
                //robotWait(5.0);
                if (String.format("%s",vuMark).equals("LEFT")) {
                    return SOCKmission.LEFT;
                } else if (String.format("%s", vuMark).equals("RIGHT")) {
                    return SOCKmission.RIGHT;
                } else if (String.format("%s", vuMark).equals("CENTER")) {
                    return SOCKmission.CENTER;
                }
            }
            else {
                //telemetry.addData("VuMark", "not visible");
            }

            //telemetry.update();
        }
        return SOCKmission.CENTER;
    }
    public void shutDownVuforia(){
        relicTrackables.deactivate();
    }


    /**
     * ---   ______________   ---
     * ---   OpenCV methods   ---
     * ---   \/ \/ \/ \/ \/   ---
     */
    public double spitCenterAngleOut(int dogeCVinput) {
        double dogeAngle = 0;

        return dogeAngle = (dogeCVinput - 400.0)/800.0;
    }
    /**
    public double cryptoboxAngle(CryptoboxDetector cryptoboxDetector, SOCKmission column) {
        double Angle = 0;
        if (cryptoboxDetector.isColumnDetected()) {
            switch (column) {
                case LEFT:
                    Angle = 0.11 * cryptoboxDetector.getCryptoBoxLeftPosition() + 39.25;
                    break;
                case CENTER:
                    Angle = 0.93 * cryptoboxDetector.getCryptoBoxLeftPosition() + 52.7;
                    break;
                case RIGHT:
                    Angle = 0.045 * cryptoboxDetector.getCryptoBoxLeftPosition() + 69.6;
                    break;
            }
        }
        return Angle;
    }
*/

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
    public void stuffishable() {
        /* Placeholder... */
    }


}// End of class bracket