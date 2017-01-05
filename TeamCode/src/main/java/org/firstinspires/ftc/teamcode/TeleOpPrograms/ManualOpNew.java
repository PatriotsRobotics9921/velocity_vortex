package org.firstinspires.ftc.teamcode.TeleOpPrograms;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="Future Hendrix Op Mode", group="Tele-Operations")
//@Disabled
public class ManualOpNew extends OpMode
{
    /* Declare OpMode members. */
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

    //Important Thresholds
    static final double     WHITE_THRESHOLD = 0.19;
    static final double     turnPower = 0.28;
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;   // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");

        motorShootLeft = hardwareMap.dcMotor.get("motor shoot left");
        motorShootRight = hardwareMap.dcMotor.get("motor shoot right");

        motorBelt = hardwareMap.dcMotor.get("motor belt");
        motorLift = hardwareMap.dcMotor.get("motor lift");

        motorCollector = hardwareMap.dcMotor.get("motor collector");

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorShootLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBelt.setDirection(DcMotor.Direction.REVERSE);
        motorCollector.setDirection(DcMotor.Direction.REVERSE);

        servoButtonClick = hardwareMap.servo.get("servo button click");
        servoExtendCollectionLeft = hardwareMap.servo.get("servo exl");
        servoExtendCollectionRight =hardwareMap.servo.get("servo exr");
        servoLiftAngle = hardwareMap.servo.get("servo lift angle");
        servoLiftGate = hardwareMap.servo.get("servo lift gate");

        servoButtonClick.setPosition(0.43);
        servoLiftGate.setPosition(0.36);
        servoLiftAngle.setPosition(0.52);
        servoExtendCollectionLeft.setPosition(0.40);
        servoExtendCollectionRight.setPosition(0.60);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        float right = gamepad1.right_stick_y;
        float left = gamepad1.left_stick_y;
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)(scaleInput(right));
        left =  (float)scaleInput(left);

        motorBackLeft.setPower(left);
        motorBackRight.setPower(right);

        if (gamepad1.a)
        {
            motorBelt.setPower(0);
        }
        else if (gamepad1.b)
        {
            motorBelt.setPower(1.0);
        }
        if (gamepad1.x)
        {
            motorShootLeft.setPower(0);
            motorShootRight.setPower(0);
        }
        else if (gamepad1.y)
        {
            motorShootLeft.setPower(1);
            motorShootRight.setPower(1);
        }
        if (gamepad2.a)
        {
            motorCollector.setPower(0);
        }
        else if (gamepad2.b)
        {
            motorCollector.setPower(1);
        }

        if(gamepad2.x)
        {
            motorLift.setPower(-1);
        }
        if(gamepad2.y)
        {
            motorLift.setPower(1);
        }
        if(gamepad2.dpad_down)
        {
            motorLift.setPower(0);

        }

        if (gamepad2.left_bumper)
        {
            servoButtonClick.setPosition(0.20);
        }
        else if (gamepad2.right_bumper)
        {
            servoButtonClick.setPosition(0.70);
        }
        else
        {
            servoButtonClick.setPosition(0.43);
        }

        if(gamepad2.dpad_left)
        {
            servoExtendCollectionLeft.setPosition(0.40);
            servoExtendCollectionRight.setPosition(0.60);
        }
        if(gamepad2.dpad_up)
        {
            servoExtendCollectionLeft.setPosition(0.50);
            servoExtendCollectionRight.setPosition(0.50);

        }
        if(gamepad2.dpad_right)
        {
            servoExtendCollectionLeft.setPosition(1.00);
            servoExtendCollectionRight.setPosition(0.00);
        }

        if(gamepad1.dpad_up)
        {
            servoLiftAngle.setPosition(0.52);
        }
        if(gamepad1.dpad_down)
        {
            servoLiftAngle.setPosition(0.00);
        }

        if(gamepad1.dpad_left)
        {
            servoLiftGate.setPosition(0.36);
        }
        if(gamepad1.dpad_right)
        {
            servoLiftGate.setPosition(0.73);
        }
    }

    @Override
    public void stop() {
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
    
}
