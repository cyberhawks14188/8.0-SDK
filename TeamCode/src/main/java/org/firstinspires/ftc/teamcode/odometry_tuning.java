package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;

@TeleOp
@Config
public class odometry_tuning extends LinearOpMode {
    public static double trackwidth = 12.5;
    double IMU;
    @Override


    public void runOpMode(){


        TestHardware robot = new TestHardware();
        OdometryCode ODO = new OdometryCode();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            IMU = -angles.firstAngle;
            if(IMU < 0){
                IMU = IMU + 360;
            }


            //telemetry.addData("imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.addData("heading", angles.firstAngle);
            ODO.TrackWidth = trackwidth;

            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorVR.getCurrentPosition(), robot.MotorHL.getCurrentPosition());


            telemetry.addData("ParaDist", ODO.ParaDist);
            telemetry.addData("PerpDist", ODO.PerpDist);
            telemetry.addData("E1",robot.MotorVL.getCurrentPosition());
            telemetry.addData("E2", robot.MotorVR.getCurrentPosition());
            telemetry.addData("E3", robot.MotorHL.getCurrentPosition());
            telemetry.addData("HeadingRAD", ODO.HeadingRAD);
            telemetry.addData("HeadingDEG", ODO.HeadingDEG);
            telemetry.addData("E1 IN", ODO.ParaLeftEncoder);
            telemetry.addData("E2 IN", ODO.PerpEncoder);
            telemetry.addData("E3 IN", ODO.ParaRightEncoder);
            telemetry.update();


        }
    }



}
