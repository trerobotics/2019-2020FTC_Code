package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//toggleable button that will make the robot go at a slower speed when it is toggled on


public class Robot
{

    /* local Setup Stuff */
    private HardwareMap hwMap           =  null;
    // private ElapsedTime period  = new ElapsedTime();

    //slow robot toggle
    public boolean slowRobot = false;

    // Drive Motors
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;


    // Robot constants

    //  private final float P_COEF = .01f;
    private final float SLOW_SPEED_SCALE = .3f;
    private float maxSpeed = 1f;

    /*
     * Imu variables
     */
    public BNO055IMU imu = null;

    /*
     * PID controller for chassis rotation.
     */
    //PIDController pidRotate;

    /* Constructor */
    public Robot(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init()
    {

        leftFrontDrive = hwMap.dcMotor.get("driveLF");
        rightFrontDrive = hwMap.dcMotor.get("driveRF");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    /*
     * only take the largest value and return that.
     * used for drive train simplification
     */
    private double absMax(double a, double b) { //Returns the argument whose absolute value is greater (similar to Math.max() but compares absolute values)
        return (Math.abs(a) > Math.abs(b)) ? a : b;
    }


    //code to be used when a button press needs to be checked
    public void check(boolean bButton)
    {
        if (bButton == true) {
            if (slowRobot == false)
            {
                maxSpeed = 0f;
                slowRobot = true;
            } else {
                maxSpeed = 1f;
                slowRobot = false;
            }
        }
    }


    // makes the robot move. Takes in three axis for movement, and a boolean for whether the robot should keep its
    // current heading. WIthOUT SLOW SPEED. USED FOR STUFF LIKE AUTONOMOUS.
    public void drive(float xInput, float yInput, float zInput, boolean keepheading)
    {

        if(keepheading)
        {
            rightFrontDrive.setPower(Range.clip(yInput - xInput - (zInput), -maxSpeed, maxSpeed));
            leftFrontDrive.setPower(Range.clip(yInput + xInput + (zInput), -maxSpeed, maxSpeed));

        } else
        {
            rightFrontDrive.setPower(Range.clip(yInput - xInput - zInput, -maxSpeed, maxSpeed));
            leftFrontDrive.setPower(Range.clip(yInput + xInput + zInput, -maxSpeed, maxSpeed));
        }
    }

}
