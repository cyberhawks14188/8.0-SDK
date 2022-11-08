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
    public DcMotor MotorVL;//port0
    public DcMotor MotorVR;//port2
    public DcMotor MotorHL;//port1
    public DcMotor MotorHR;//port3
    public DcMotorEx MotorLift;
    public CRServo IntakeS;

    /*BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;*/






    HardwareMap testhardware;

    public void init(HardwareMap testhardware){

     /*   BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = testhardware.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);*/



        // Define motors and servos
        IntakeS = testhardware.get(CRServo.class, "IntakeS");
        MotorVL = testhardware.get(DcMotorEx.class, "MotorVL");
        MotorVR = testhardware.get(DcMotorEx.class, "MotorVR");
        MotorHL = testhardware.get(DcMotorEx.class, "MotorHL");
        MotorHR = testhardware.get(DcMotorEx.class, "MotorHR");
        MotorLift = testhardware.get(DcMotorEx.class, "MotorLift");


        MotorVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
