package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.firstinspires.ftc.teamcode.TestHardware;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp extends LinearOpMode {
//test
    public static double tickstoin = 1775;
    public static double Trackwidth = 9.13;
    public static double VertOffset = 4.75;
    public static double zP = .00004;
    public static double zD = 0.002;

    double x, y, z;
    double finalX, finalY;
    double RLDIR, RRDIR, LRDIR, LLDIR;

    double vectorMagnitude = 0, vectorAngleRAD = 0, vectorAngleDEG = 0;
    double finalvectorAngleDEG = 0;

    double IMU = 0;

    double liftset = 0;
    double liftcurrentpos;

    double drivespeed = 1;

    double headingsetpoint = 0;

    double Maxpower = 0;

    boolean lastx = false;
    boolean lasty = false;




    TestHardware robot = new TestHardware();
    OdometryCode ODO = new OdometryCode();
    LiftControl lift = new LiftControl();
    HeadingControl HDing = new HeadingControl();
    Smoothing Smoothing = new Smoothing();
    SpeedClass SpeedClass = new SpeedClass();



    @Override

    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            SpeedClass.SpeedCalc(1,0, 0, 0, ODO.ParaDist, ODO.PerpDist, getRuntime());

            if( gamepad1.back){
                ODO.HeadingRAD = Math.toRadians(0);
                headingsetpoint = 0;
            }
            ODO.PerpOffset = VertOffset;
            ODO.TicksToInches = tickstoin;
            ODO.TrackWidth = Trackwidth;

            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());
            /*
            joysticks: used
            dpad: used
            bumpers: used
            triggers: used
            a/b/x/y: FREE
            back: FREE
            start: FREE
            joystickButtons: FREE
            */



            //lift code
            liftcurrentpos = robot.MotorLift.getCurrentPosition();

            if(gamepad1.dpad_down){
                liftset = 0;
            }else if(gamepad1.dpad_left){
                liftset = 680;
            }else if(gamepad1.dpad_right){
                liftset = 1100;
            }else if(gamepad1.dpad_up){
                liftset = 1480;
            }else{
                if(gamepad1.left_bumper){
                    liftset += 25 * -gamepad1.right_stick_y;
                }
            }

            if(gamepad1.x && lastx == false){
                liftset = liftset - 100;
            }

            if(gamepad1.x){
                lastx = true;
            }else{
                lastx = false;
            }

            if(gamepad1.y && lasty == false){
                liftset = liftset + 100;
            }

            if(gamepad1.y){
                lasty = true;
            }else{
                lasty = false;
            }


            lift.LiftMethod(liftset, 1500, robot.MotorLift.getCurrentPosition(), getRuntime());


            robot.MotorLift.setPower(lift.liftpower);

            //intake code
            
            if(gamepad1.a || gamepad1.left_trigger > .05){
                robot.IntakeS.setPower(.5);
            }else if(gamepad1.b){
                robot.IntakeS.setPower(-.5);
            }else{
                robot.IntakeS.setPower(0.05);
            }

            //IMU drive code

            /*Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            IMU = -angles.firstAngle;
            if(IMU < 0){
                IMU = IMU + 360;
            }*/

            //drive code

            x = Smoothing.SmoothPerpendicularInput(Math.copySign(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_x));

            y = -Smoothing.SmoothParallelInput(Math.copySign(gamepad1.left_stick_y * gamepad1.left_stick_y, gamepad1.left_stick_y));
            //z = gamepad1.right_stick_x;
            if (vectorAngleDEG < 0) {
                vectorAngleDEG = vectorAngleDEG + 360;
            }

            //defines the vector
            //angle should output 0-360 deg
            vectorMagnitude = Math.sqrt((y * y) + (x * x));//hypotenuse of the joysticks
            vectorAngleRAD = Math.atan2(y, x);//angle of the joystick inputs in Radians
            vectorAngleDEG = -Math.toDegrees(vectorAngleRAD);//converts that into degrees


            //Subtracts the heading angle to create the virtual forward
            finalvectorAngleDEG = vectorAngleDEG - ODO.HeadingDEG;

            //convert back into x and y values
            finalX = vectorMagnitude * Math.cos(Math.toRadians(finalvectorAngleDEG));
            finalY = vectorMagnitude * Math.sin(Math.toRadians(finalvectorAngleDEG));

            if(gamepad1.right_bumper){
                drivespeed = .4;
                if(!gamepad1.left_bumper){
                    headingsetpoint += Smoothing.SmoothHeadingInput(Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x)) * 2.5;
                }
            }else if(gamepad1.right_trigger > .1) {
                drivespeed = 1;
                headingsetpoint += Smoothing.SmoothHeadingInput(Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x)) * 5;
            }else{
                drivespeed = .7;
                if(!gamepad1.left_bumper){
                    headingsetpoint += Smoothing.SmoothHeadingInput(Math.copySign(gamepad1.right_stick_x * gamepad1.right_stick_x, gamepad1.right_stick_x)) * 5;
                }
            }

            HDing.headingP = zP;
            HDing.headingD = zD;

            HDing.HeadingMethod(headingsetpoint, 350, ODO.HeadingDEG, getRuntime());

            z = HDing.headingPower;

            LLDIR = -finalY + z;
            LRDIR = finalY + z;
            RLDIR = -finalX + z;
            RRDIR = finalX + z;






            robot.MotorVL.setPower(LLDIR * drivespeed);
            robot.MotorVR.setPower(LRDIR * drivespeed);
            robot.MotorHL.setPower(RLDIR * drivespeed);
            robot.MotorHR.setPower(RRDIR * drivespeed);

            dashboardTelemetry.addData("heading", ODO.HeadingDEG);
            dashboardTelemetry.addData("headingSet", headingsetpoint);
            dashboardTelemetry.update();

            telemetry.addData("speed", SpeedClass.currentSpeed);
            telemetry.addData("heading current speed", HDing.headingCurrentSpeed);
            telemetry.addData("inmotionprofile?", HDing.inmotionprofileheading);
            telemetry.addData("heading set", headingsetpoint);
            telemetry.addData("heading", ODO.HeadingDEG);
            telemetry.addData("heading power", HDing.headingPower);
            telemetry.addData("para in", ODO.ParaDist);
            telemetry.addData("perp in", ODO.PerpDist);
            telemetry.addData("Encoder Para Left", robot.MotorVL.getCurrentPosition());
            telemetry.addData("Encoder Para Right", robot.MotorVR.getCurrentPosition());
            telemetry.addData("Encoder Perp", robot.MotorHL.getCurrentPosition());

            telemetry.addData("IMU v", IMU);
            telemetry.addData("liftset", liftset);
            telemetry.addData("lift power", lift.liftpower);
            telemetry.addData("liftpos", liftcurrentpos);


            telemetry.update();

        }
    }
}
