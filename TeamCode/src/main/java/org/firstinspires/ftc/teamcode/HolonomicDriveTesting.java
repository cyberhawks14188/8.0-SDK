package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;

@TeleOp

public class HolonomicDriveTesting extends LinearOpMode {

    double x, y, z;
    double finalX, finalY;
    double RLDIR, RRDIR, LRDIR, LLDIR;

    double vectorMagnitude = 0, vectorAngleRAD = 0, vectorAngleDEG = 0;
    double finalvectorAngleDEG = 0;



    TestHardware robot = new TestHardware();
    OdometryCode ODO = new OdometryCode();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override

    public void runOpMode() {


        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.Servo1.setPower(.4);
                robot.Servo2.setPower(-.4);
            } else if (gamepad1.b) {
                robot.Servo1.setPower(-.4);
                robot.Servo2.setPower(.4);
            } else {
                robot.Servo1.setPower(0);
                robot.Servo2.setPower(0);
            }

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            //defines the vector
            //angle should output 0-360 deg

            vectorMagnitude = Math.sqrt((y * y) + (x * x));
            vectorAngleRAD = Math.atan2(y, x);
            vectorAngleDEG = -Math.toDegrees(vectorAngleRAD);
            if (vectorAngleDEG < 0) {
                vectorAngleDEG = vectorAngleDEG + 360;
            }

            //add in angle offset
            finalvectorAngleDEG = vectorAngleDEG + ODO.HeadingDEG;

            //convert back into x and y values
            finalX = vectorMagnitude * Math.cos(Math.toRadians(finalvectorAngleDEG));
            finalY = vectorMagnitude * Math.sin(Math.toRadians(finalvectorAngleDEG));


            LLDIR = -finalY + z;
            LRDIR = finalY + z;
            RLDIR = -finalX + z;
            RRDIR = finalX + z;


            robot.MotorVL.setPower(LLDIR);
            robot.MotorVR.setPower(LRDIR);
            robot.MotorHL.setPower(RLDIR);
            robot.MotorHR.setPower(RRDIR);


            dashboardTelemetry.update();

            telemetry.addData("heading", ODO.HeadingDEG);

            telemetry.update();

        }
    }
}
