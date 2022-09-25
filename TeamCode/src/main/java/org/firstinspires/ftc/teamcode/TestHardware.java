package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class TestHardware {
    // Motors and Servos
    public DcMotor MotorVL;
    public DcMotor MotorVR;
    public DcMotor MotorHL;
    public DcMotor MotorHR;
    public CRServo Servo1;
    public CRServo Servo2;
    public DistanceSensor Dist1;
    public DistanceSensor Dist2;
    public DistanceSensor Dist3;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;






    HardwareMap testhardware;

    public void init(HardwareMap testhardware){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = testhardware.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);



        Dist1 = testhardware.get(DistanceSensor.class, "Dist1");
        Dist2 = testhardware.get(DistanceSensor.class, "Dist2");
        Dist3 = testhardware.get(DistanceSensor.class, "Dist3");



        // Define motors and servos
        MotorVL = testhardware.get(DcMotorEx.class, "Motor1");
        MotorVR = testhardware.get(DcMotorEx.class, "Motor2");
        MotorHL = testhardware.get(DcMotorEx.class, "Motor3");
        MotorHR = testhardware.get(DcMotorEx.class, "Motor4");
        Servo1 = testhardware.get(CRServo.class, "CRServo1");
        Servo2 = testhardware.get(CRServo.class, "CRServo2");


        MotorVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       /* MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
