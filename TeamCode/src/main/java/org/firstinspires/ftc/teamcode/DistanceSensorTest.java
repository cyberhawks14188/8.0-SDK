package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class DistanceSensorTest extends LinearOpMode {
//test
    TestHardware robot = new TestHardware();

    @Override

    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Dist1", robot.Dist1.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist2", robot.Dist2.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist3", robot.Dist3.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist4", robot.Dist4.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist5", robot.Dist5.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist6", robot.Dist6.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist7", robot.Dist7.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dist8", robot.Dist8.getDistance(DistanceUnit.INCH));
            telemetry.update();
            dashboardTelemetry.addData("Dist1", robot.Dist1.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist2", robot.Dist2.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist3", robot.Dist3.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist4", robot.Dist4.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist5", robot.Dist5.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist6", robot.Dist6.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist7", robot.Dist7.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.addData("Dist8", robot.Dist8.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.update();


        }
    }
}
