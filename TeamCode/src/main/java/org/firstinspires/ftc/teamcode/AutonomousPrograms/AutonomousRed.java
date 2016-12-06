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

@Autonomous(name="Full AAutonomous RED", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutonomousRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorShootLeft;
    DcMotor motorShootRight;
    DcMotor motorBelt;
    DcMotor motorCollector;
    DcMotor motorLift;

    Servo servoClawLeft;
    Servo servoClawRight;
    Servo servoButtonClick;
    Servo servoClamp;

    DeviceInterfaceModule cdim;
    ModernRoboticsI2cGyro sensorGyro;
    LightSensor lightSensor;

    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    static final double     WHITE_THRESHOLD = 0.19;
    double highPower = 0.70;
    double medPower = 0.20;
    double lowPower = 0.10;
    double turnPower = 0.28;

    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;   // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode ()  throws InterruptedException
    {
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        motorShootLeft = hardwareMap.dcMotor.get("motor shoot left");
        motorShootRight = hardwareMap.dcMotor.get("motor shoot right");
        motorBelt = hardwareMap.dcMotor.get("motor belt");
        motorCollector = hardwareMap.dcMotor.get("motor collector");
        motorLift = hardwareMap.dcMotor.get("motor lift");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShootRight.setDirection(DcMotor.Direction.REVERSE);
        motorBelt.setDirection(DcMotor.Direction.REVERSE);

        servoClawLeft = hardwareMap.servo.get("servo claw left");
        servoClawRight = hardwareMap.servo.get("servo claw right");
        servoButtonClick = hardwareMap.servo.get("servo button click");
        servoClamp = hardwareMap.servo.get("servo clamp");

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensor gyro");
        lightSensor = hardwareMap.lightSensor.get("sensor light");
//the below lines set up the configuration file
        colorC = hardwareMap.i2cDevice.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        servoClawLeft.setPosition(.20);
        servoClawRight.setPosition(.80);
        servoButtonClick.setPosition(0.23);

        sensorSetup();
        colorCreader.write8(3, 1); //set led off
        colorCreader.write8(3, 0); //set led on

        //Vuforia setup
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

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //telemetry.addData("Light Level", lightSensor.getLightDetected());
        //telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            encoderDrive(.2, -7,-7);
            shootShooters();
            turn(91.0,"left",turnPower,0);
            while(opModeIsActive() && wheels.getRawPose() == null)
            {
                move(0.8,0.8);
            }
            move(0,0);

            VectorF angles = anglesFromTarget(wheels);
            VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));

            double ogTrans = trans.get(2);
            double scaledPower;
            //get to 50mm off wall
            while(opModeIsActive() && trans.get(2) > 0)
            {
                scaledPower = ((ogTrans - trans.get(2)) /(ogTrans));
                if (scaledPower < 0.2)
                    scaledPower = 0.2;
                move(scaledPower,scaledPower);
                angles = anglesFromTarget(wheels);
                trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));
            }
            //turn to be perpendicular to wall
            while (opModeIsActive() && Math.abs(Math.toDegrees(angles.get(0)) - 90) != 90)
            {
                //Turn left until 90
                move(-0.24,0.24);
            }
            move(0,0);
            analyzeBeacon();
            turn(87,"right",turnPower,0);
            encoderDrive(1.0,10,10);
            seeWhiteLine();
            turn(87,"left",turnPower,0);
            analyzeBeacon();
            sleep(30000);
            //Autonomous finished (sort of)
        }
    }

    public void driveEncoders()
    {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition());
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

    /*
        public void wallDetection()
        {

            while (ultraThugga.getUltrasonicLevel() > 12)
            {
                move(medPower,medPower);
                telemetry.addData("Ultrasonic", ultraThugga.getUltrasonicLevel());
                waitForNextHardwareCycle();
            }

            move(0,0);
        }
        */
    public void sensorSetup()
    {
        lightSensor.enableLed(true);

        telemetry.addData(">", "Gyro Calibrating. Do not move!");
        telemetry.update();
        sensorGyro.calibrate();
        sleep(500);
        telemetry.update();
        telemetry.addData("Text", "Sensors Ready");
        telemetry.update();
    }


    public void seeWhiteLine()
    {
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD)
        {
            move(0.20,0.20);
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

    /*public void turnRight(int Degrees)
    {
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorGyro.resetZAxisIntegrator();
        sleep(500);

        while (Math.abs(sensorGyro.getIntegratedZValue()) < Degrees)
        {
            move(turnPower, -turnPower);
            telemetry.addData("Heading Degrees : ", sensorGyro.getIntegratedZValue());
            telemetry.addData("Direction : ", "Turning Right");
            waitForNextHardwareCycle();
        }
        move(0,0);
        telemetry.addData("Finished!", "Turning point reached");
        sleep(1000);
    }

    public void turnLeft(int Degrees)
    {
        sensorGyro.resetZAxisIntegrator();
        sleep(500);

        while (Math.abs(sensorGyro.getIntegratedZValue()) < Degrees)
        {
            move(-turnPower, turnPower);
            telemetry.addData("Heading Degrees : ", sensorGyro.getIntegratedZValue());
            telemetry.addData("Direction : ", "Turning Left");
            waitForNextHardwareCycle();
        }
        move(0,0);
        telemetry.addData("Finished!", "Turning point reached");
        sleep(1000);
    }*/

    public void timedMove(double power, int milliseconds)
    {
        move(power,power);
        sleep(milliseconds);
        move(0,0);
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

    public void analyzeBeacon()
    {
       if ((colorCcache[0] & 0xFF) < 1)
       {
           colorCcache = colorCreader.read(0x04, 1);
           telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
           //extend a servo
       }
       else
       {
           colorCcache = colorCreader.read(0x04, 1);
           telemetry.addData("Color Number: ", colorCcache[0] & 0xFF);
           //extend other servo
       }
    }
}
