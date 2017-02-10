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

@TeleOp(name="Holonomic New", group="Tele-Operations")
//@Disabled
public class HolonomicDrive extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    DcMotor frontLeft = null,
            frontRight = null,
            backLeft = null,
            backRight = null,

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);



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

        //assign motor powers their inputs based on position on the robot
        flPwr = Range.clip((+movX + movY), -1, 1) + rot;
        frPwr = Range.clip((-movX + movY), -1, 1) - rot;
        blPwr = Range.clip((-movX + movY), -1, 1) + rot;
        brPwr = Range.clip((+movX + movY), -1, 1) - rot;

        //balance movement and rotation
        brPwr -= Math.max(flPwr-1, 0) - Math.min(flPwr+1, 0);
        blPwr -= Math.max(frPwr-1, 0) - Math.min(frPwr+1, 0);
        frPwr -= Math.max(blPwr-1, 0) - Math.min(blPwr+1, 0);
        flPwr -= Math.max(brPwr-1, 0) - Math.min(brPwr+1, 0);

        frontLeft.setPower(flPwr);
        frontRight.setPower(frPwr);
        backLeft.setPower(blPwr);
        backRight.setPower(brPwr);

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
