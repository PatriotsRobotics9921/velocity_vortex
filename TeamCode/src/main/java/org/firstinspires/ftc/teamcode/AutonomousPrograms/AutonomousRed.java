package org.firstinspires.ftc.teamcode.AutonomousPrograms;

import android.graphics.Bitmap;
import android.util.Log;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import java.util.Arrays;

@Autonomous(name="Autonomous Red ", group="Autonomous")
//@Disabled
public class AutonomousRed extends LinearOpMode {

    //Timer
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorShootLeft;
    DcMotor motorShootRight;
    DcMotor motorBelt;
    DcMotor motorCollector;
    DcMotor motorLift;

    //Servos
    Servo servoButtonClick;
    Servo servoExtendCollectionLeft;
    Servo servoExtendCollectionRight;
    Servo servoLiftGate;
    Servo servoLiftAngle;

    //Sensors
    DeviceInterfaceModule cdim;
    ModernRoboticsI2cGyro sensorGyro;
    LightSensor lightSensor;
    OpticalDistanceSensor sensorOptical;

        //Color Sensor (Beacon)
        byte[] colorCcache;
        I2cDevice colorC;
        I2cDeviceSynch colorCreader;

    //Important Thresholds
    static final double     WHITE_THRESHOLD = 0.37;
    static final double     turnPower = 0.28;
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;   // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to
    //Raw value is between 0 and 1
    double odsReadingRaw;
    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;


    @Override
    public void runOpMode ()  throws InterruptedException
    {
        ///////////////////////////////////HARDWARE MAP//////////////////////////////////////////////////
        //Hardware Map for Motors
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");

        motorShootLeft = hardwareMap.dcMotor.get("motor shoot left");
        motorShootRight = hardwareMap.dcMotor.get("motor shoot right");

        motorBelt = hardwareMap.dcMotor.get("motor belt");
        motorLift = hardwareMap.dcMotor.get("motor lift");

        motorCollector = hardwareMap.dcMotor.get("motor collector");
        //motorCollectorHub = hardwareMap.dcMotor.get("motor collector hub");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShootRight.setDirection(DcMotor.Direction.REVERSE);

        //Hardware Map for Servos
        servoButtonClick = hardwareMap.servo.get("servo button click");
        servoExtendCollectionLeft = hardwareMap.servo.get("servo exl");
        servoExtendCollectionRight =hardwareMap.servo.get("servo exr");
        servoLiftAngle = hardwareMap.servo.get("servo lift angle");
        servoLiftGate = hardwareMap.servo.get("servo lift gate");

        //Hardware Map for Sensors
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensor gyro");
        lightSensor = hardwareMap.lightSensor.get("sensor light");
        //sensorOptical = hardwareMap.opticalDistanceSensor.get("ods");
        colorC = hardwareMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        ///////////////////////////////////HARDWARE MAP//////////////////////////////////////////////////

        ///////////////////////////////////INITIALIZE MAP//////////////////////////////////////////////////
        //Servo Set-Up Initialize
        //Servo Set-Up Initialize
        servoButtonClick.setPosition(0.43);
        servoLiftGate.setPosition(0.36);
        servoLiftAngle.setPosition(0.52);
        servoExtendCollectionLeft.setPosition(0.40);
        servoExtendCollectionRight.setPosition(0.60);

        sensorSetup();
        ///////////////////////////////////INITIALIZE MAP//////////////////////////////////////////////////

        ///////////////////////////////VUFORIA Set-Up//////////////////////////////////////////////////
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AY6rtrH/////AAAAGVWmuhBqk0JTpGx6FODTktFXl6LaoSyhRtq+O84NCurm5LfqvbhYaEq5tEDNLZ9utp4EoD3hyGC938qXyd68xFVepamsJTiZElIwEoTeBF4Bxgk14/y++c1W4oyGOqnVI+DVwBRWbQmqQ4myF5Y2wwXLdo3FjYP+lHvSI0BScISNhkZmJlWhb7PgZfJ+bmTWOg7vbSaZeH+yHoUKBeGBv5w+AlrCSdu2rYgro4Nu5T75IXzFvI0jT17+DLEoZpe/QgpYoAmoBbswfClsVi8rAJay63RS2uHYJ/HHNyXKaAzkVG3S94CWj6zQT5QJKoko2Ox5iyt6GTKXGWwmmzQGzmluGtaidX7YFfCGsUfYpiuw";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        VuforiaTrackableDefaultListener vuforiaMain = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        beacons.activate();
        ///////////////////////////////VUFORIA Set-Up//////////////////////////////////////////////////



        /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////MAIN AUTONOMOUS CODE END//////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
        colorCcache = colorCreader.read(0x04, 1);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            colorCreader.write8(3, 1); //set led off
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            telemetry.update();

            encoderDrive(1, 1, 1);
            turn(40.0,"left",turnPower,0);
            seeWhiteLine();
            encoderDrive(0.5, -0.5, -0.5);
            turn(49.5,"left",turnPower,0);
            encoderDrive(0.2, 2,2);
            analyzeBeacon();
            encoderDrive(0.5, -7,-7);
            shootShooters();
            encoderDrive(1, -7, -7);
            turn(15,"right",turnPower,0);
            encoderDrive(1, -3, -3);
            sleep(20000);
        }
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////MAIN AUTONOMOUS CODE END//////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/



    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////ENCODER FUNCTIONS/////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    public void driveEncoders()
    {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d", motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches)   {
        driveEncoders();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motorBackLeft.setTargetPosition(newLeftTarget);
            motorBackRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (motorBackLeft.isBusy() && motorBackRight.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorBackLeft.getCurrentPosition(),
                        motorBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);
        }
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////ENCODER FUNCTIONS/////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////SENSORS //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    public void sensorSetup()
    {
        colorCreader.write8(3, 1); //set led off
        colorCreader.write8(3, 0); //set led on
        colorCreader.write8(3, 1); //set led off

        lightSensor.enableLed(true);
        gyroSetUp();
    }

    public void seeWhiteLine()
    {
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD)
        {
            move(.3,.3);
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();

        }
        telemetry.addData("WHITE LINE FOUND!", lightSensor.getLightDetected());
        move(0,0);
    }

    public void seeWhiteLineBack()
    {
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD)
        {
            move(-0.18,-0.18);
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
        }
        telemetry.addData("WHITE LINE FOUND!", lightSensor.getLightDetected());
        move(0,0);
    }

    public void move(double powerLeft,double powerRight)
    {
        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }

    public void timedMove(double power, int milliseconds)
    {
        move(power,power);
        sleep(milliseconds);
        move(0,0);
    }


    public void wallFollowWhiteLine()
    {
        while(lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            odsReadingRaw = sensorOptical.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, 0.5);                //calculate linear value

            motorBackLeft.setPower(odsReadingLinear * 2);
            motorBackRight.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 Motor Left", motorBackLeft.getPower());
            telemetry.addData("3 Motor Right", motorBackRight.getPower());
            telemetry.update();
        }
    }
    public void analyzeBeacon()
    {
        colorCcache = colorCreader.read(0x04, 1);

        if ((colorCcache[0] & 0xFF) > 7 && (colorCcache[0] & 0xFF) < 13) //IF RED
        {
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            servoButtonClick.setPosition(0.80);
            encoderDrive(1,2,2);
        }
        else   //BLUE
        {
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            servoButtonClick.setPosition(0.10);
            encoderDrive(1,2,2);
        }
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////SENSORS //////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////VUFORIA///////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall)
    {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image)
    {
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////VUFORIA///////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/


    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////GYRO FUNCTIONS////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    public void driveStraightGyro(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = sensorGyro.getIntegratedZValue();  //Starting direction
        double startPosition = motorBackLeft.getCurrentPosition();  //Starting position

        while (motorBackLeft.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            zAccumulated = sensorGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            motorBackLeft.setPower(leftSpeed);
            motorBackRight.setPower(rightSpeed);

            telemetry.addData("1. Left", motorBackLeft.getPower());
            telemetry.addData("2. Right", motorBackRight.getPower());
            telemetry.addData("3. Distance to go", duration + startPosition - motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target)  {
        turnAbsolute(target + sensorGyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = sensorGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                motorBackLeft.setPower(turnSpeed);
                motorBackRight.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                motorBackLeft.setPower(-turnSpeed);
                motorBackRight.setPower(turnSpeed);
            }

            zAccumulated = sensorGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        motorBackLeft.setPower(0);  //Stop the motors
        motorBackRight.setPower(0);

    }

    public void gyroSetUp()
    {
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        sensorGyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && sensorGyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
    }

    private void turn(double degrees, String direction, double maxSpeed, int count)
    {
        if (!opModeIsActive()) return;
        if (direction.equals("right")) degrees *= -1; //Negative degree for turning right
        double targetHeading = sensorGyro.getIntegratedZValue() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0)//right
        {
            while (sensorGyro.getIntegratedZValue() > targetHeading && opModeIsActive())
            {
                motorBackLeft.setPower(Range.clip(maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
                motorBackRight.setPower(Range.clip(-maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));

                telemetry.addData("Distance to turn: ", Math.abs(sensorGyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", sensorGyro.getIntegratedZValue());
                telemetry.addData("Original Speed", maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)));
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));

                telemetry.update();
            }
        }
        else //Left
        {
            while (sensorGyro.getIntegratedZValue() < targetHeading && opModeIsActive())
            {
                motorBackLeft.setPower(Range.clip(-maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));
                motorBackRight.setPower(Range.clip(maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(sensorGyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", sensorGyro.getIntegratedZValue());
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));

                telemetry.update();
            }
        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(200);

        telemetry.addData("Distance to turn", Math.abs(sensorGyro.getIntegratedZValue() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if(Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) > 0 && count < 1)
        {
            //Recurse to correct turn
            turn(Math.abs(sensorGyro.getIntegratedZValue() - targetHeading), direction.equals("right") ? "left" : "right", .08, ++count);
        }
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////GYRO FUNCTIONS////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////OTHER FUNCTIONS////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    public void shootShooters()
    {
        motorShootLeft.setPower(-1);
        motorShootRight.setPower(-1);
        motorBelt.setPower(-1);
        sleep(3000);
        motorShootLeft.setPower(0);
        motorShootRight.setPower(0);
        motorBelt.setPower(0);
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////OTHER FUNCTIONS////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
}
