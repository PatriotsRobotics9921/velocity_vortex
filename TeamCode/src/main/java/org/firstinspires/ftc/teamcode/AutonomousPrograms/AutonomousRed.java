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


@Autonomous(name="Autonomous Red ", group="Autonomous")
//@Disabled
public class AutonomousRed extends LinearOpMode {

    //Timer
    private ElapsedTime runtime = new ElapsedTime();

    //Motors //future
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorShootLeft;
    DcMotor motorShootRight;
    DcMotor motorBelt;
    DcMotor motorCollector;
    DcMotor motorLift;

    //Servos
    Servo servoButtonLeft;
    Servo servoButtonRight;
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
    static final double     WHITE_THRESHOLD = 0.39;
    double     prevValLight = 0.00;
    static final double     turnPower = 0.28;
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;   // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to
    double odsReadingRaw;//Raw value is between 0 and 1
    static double odsReadingLinear;    // odsReadingRaw to the power of (-0.5)

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

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShootRight.setDirection(DcMotor.Direction.REVERSE);

        //Hardware Map for Servos
        servoButtonLeft = hardwareMap.servo.get("servo button left");
        servoButtonRight = hardwareMap.servo.get("servo button right");
        servoLiftAngle = hardwareMap.servo.get("servo lift angle");
        servoLiftGate = hardwareMap.servo.get("servo lift gate");

        //Hardware Map for Sensors
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensor gyro");
        lightSensor = hardwareMap.lightSensor.get("sensor light");

        colorC = hardwareMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        ///////////////////////////////////HARDWARE MAP//////////////////////////////////////////////////

        ///////////////////////////////////INITIALIZE MAP//////////////////////////////////////////////////
        //Servo Set-Up Initialize
        servoButtonLeft.setPosition(0.53);
        servoButtonRight.setPosition(0.33);
        servoLiftGate.setPosition(0.36);
        servoLiftAngle.setPosition(0.25);

        sensorSetup();
        ///////////////////////////////////INITIALIZE MAP//////////////////////////////////////////////////

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //&& opModeIsActive
        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            colorCreader.write8(3, 1); //set led off
            colorCcache = colorCreader.read(0x04, 1);

            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            telemetry.update();

            encoderDrive(0.3, 1, 1);
            turn(46.0,"left",turnPower,0);
            encoderDrive(0.4, 10, 10);
            seeWhiteLine();
            encoderDrive(0.1, -0.5, -0.5);
            turn(49.5,"left",turnPower,0);
            timedMove(0.3, 300);
            //encoderDrive(0.1, 3,3);
            sleep(500);
            analyzeBeacon();
            encoderDrive(0.5, -10,-10);
            shootShooters();
            sleep(100000);
        }
    }

    public void driveEncoders()
    {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
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
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

    public void sensorSetup()
    {
        colorCreader.write8(3, 1); //set led off
        colorCreader.write8(3, 0); //set led on
        colorCreader.write8(3, 1); //set led off

        lightSensor.enableLed(true);
        gyroSetUp();

        //Color
        colorCcache = colorCreader.read(0x04, 1);
        telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
        telemetry.update();
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

    public void seeWhiteLine()
    {

        while (lightSensor.getLightDetected() < WHITE_THRESHOLD && opModeIsActive())
        {
            move(.2,.2);
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();

        }
        telemetry.addData("WHITE LINE FOUND!", lightSensor.getLightDetected());
        move(0,0);
    }

    public void seeWhiteLineBack()
    {
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD && opModeIsActive())
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
        move(power,(power + 0.07));
        sleep(milliseconds);
        move(0,0);
    }


    public void analyzeBeacon()
    {
        sleep(500);
        colorCcache = colorCreader.read(0x04, 1);
        sleep(500);

        if ((colorCcache[0] & 0xFF) > 7 && (colorCcache[0] & 0xFF) < 13 && opModeIsActive()) //IF RED
        {
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            servoButtonRight.setPosition(0.76);
            timedMove(0.2,700);
            sleep(1000);
            servoButtonRight.setPosition(0.33);
        }
        else   //BLUE
        {
            colorCcache = colorCreader.read(0x04, 1);
            telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
            servoButtonLeft.setPosition(0.10);
            timedMove(0.2,700);
            sleep(1000);
            servoButtonLeft.setPosition(0.53);
        }
    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////SENSORS //////////////////////////////////////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

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

        if(Math.abs(sensorGyro.getIntegratedZValue() - targetHeading) > 0 && count < 1 && opModeIsActive())
        {
            //Recurse to correct turn
            turn(Math.abs(sensorGyro.getIntegratedZValue() - targetHeading), direction.equals("right") ? "left" : "right", .08, ++count);
        }
    }

    public void shootShooters()
    {
        motorShootLeft.setPower(-1);
        motorShootRight.setPower(-1);
        motorBelt.setPower(-1);
        sleep(2800);
        motorShootLeft.setPower(0);
        motorShootRight.setPower(0);
        motorBelt.setPower(0);
    }
}
