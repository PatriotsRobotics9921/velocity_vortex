
package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Servo Position Acquirement", group = "Tests")
//@Disabled
public class ServoTest extends OpMode {

    Servo servo;

    // position of the arm servo.
    double servoPos;

    // amount to change the arm servo position.
    double servoDelta = 0.01;


    public ServoTest(){}

    @Override
    public void init() {

        servo = hardwareMap.servo.get("servo");


        servoPos = 0.5;
        servo.setPosition(servoPos);
    }

    @Override
    public void loop()
    {

        if(gamepad1.left_bumper) {
            servoPos -= servoDelta;
        }
        if (gamepad1.right_bumper){
            servoPos += servoDelta;
        }

        servoPos = Range.clip(servoPos, -1, 1);

        // write position values to the wrist and claw servo
        servo.setPosition(servoPos);

        telemetry.addData("Servo Position", "Servo:  " + String.format("%.2f", servoPos));
        telemetry.update();

    }

    @Override
    public void stop() {}

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