package org.firstinspires.ftc.teamcode.tests;

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

@Autonomous(name="Test", group="Autonomous")
//@Disabled
public class vuforiaTest extends LinearOpMode {

    //Timer
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode ()  throws InterruptedException
    {
        ///////////////////////////////////HARDWARE MAP//////////////////////////////////////////////////
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {

            while(opModeIsActive()) {

                try
                {
                    vuforiaMain = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
                    VectorF angles = anglesFromTarget(vuforiaMain);
                    VectorF trans = navOffWall(vuforiaMain.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));
                    angles = anglesFromTarget(vuforiaMain);
                    trans = navOffWall(vuforiaMain.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

                    telemetry.addData("Angle", angles.get(1));
                    telemetry.addData("Distance from Wall", trans.get(2));
                    telemetry.update();
                }
                catch (NullPointerException npe)
                {
                    // It's fine if findUser throws a NPE
                }
            }

        }


    }

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////MAIN AUTONOMOUS CODE END//////////////////////////////////////////////////////////
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

}
