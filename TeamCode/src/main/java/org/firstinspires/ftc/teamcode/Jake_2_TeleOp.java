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

public class Jake_2_TeleOp extends LinearOpMode {
//test
    public static double tickstoin = 1825;
    public static double Trackwidth = 8.35;
    public static double VertOffset = 4.75;
    public static double zP = .0006;
    public static double zD = 0.002;
    public static double LIFTP = .0003;
    public static double LIFTSPEEDSET = 1500;

    double x, y, z;
    double finalX, finalY;
    double RLDIR, RRDIR, LRDIR, LLDIR;
    double liftSpeedSet = 1500;
    double lastIntakeSensor = 10;
    boolean wasIntakeOpen = true;

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

    double alignmentBarSet = 0.99;
    boolean lastStart = false;




    Jake_2_Hardware robot = new Jake_2_Hardware();
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
            HDing.inTeleOp = true;
            lift.isJake2 = true;

            lift.liftP = LIFTP;

            //liftSpeedSet = LIFTSPEEDSET;
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

            if(gamepad1.start && !lastStart){
                lastStart = true;
                if(alignmentBarSet != .65){
                    alignmentBarSet = .65;
                }else{
                    alignmentBarSet = .99;
                }
            }

            if(!gamepad1.start){
                lastStart = false;
            }


            robot.AlignmentBar.setPosition(alignmentBarSet);

            //lift code
            liftcurrentpos = -robot.MotorLift.getCurrentPosition();

            if(gamepad1.dpad_down){
                liftset = 0;
            }else if(gamepad1.dpad_left){
                liftset = 500;
            }else if(gamepad1.dpad_right){
                liftset = 825;
            }else if(gamepad1.dpad_up) {
                liftset = 1125;
            }else{
                if(gamepad1.left_bumper){
                    liftset += 15 * -gamepad1.right_stick_y;
                }
            }

            if(wasIntakeOpen == true && robot.IntakeV3.getDistance(DistanceUnit.INCH) < 1.5){
                liftset = liftset + 50;
                wasIntakeOpen = false;
            }

            if(robot.IntakeV3.getDistance(DistanceUnit.INCH) > 1.5){
                wasIntakeOpen = true;
            }


            if(gamepad1.x && lastx == false){
                liftset = liftset - 75;
            }

            if(gamepad1.x){
                lastx = true;
            }else{
                lastx = false;
            }

            if(gamepad1.y && lasty == false){
                liftset = liftset + 75;
            }

            if(gamepad1.y){
                lasty = true;
            }else{
                lasty = false;
            }

            if(liftset < liftcurrentpos){
                liftSpeedSet = 800;
            }else{
                liftSpeedSet = 1500;
            }




            lift.LiftMethod(liftset, liftSpeedSet, liftcurrentpos, getRuntime());


            robot.MotorLift.setPower(lift.liftpower);

            //intake code
            
            if(gamepad1.a || gamepad1.left_trigger > .05){
                robot.IntakeS.setPower(.5);
            }else if(gamepad1.b){
                robot.IntakeS.setPower(-.5);
            }else{
                robot.IntakeS.setPower(0.1);
            }

            //IMU drive code

            /*Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            IMU = -angles.firstAngle;
            if(IMU < 0){
                IMU = IMU + 360;
            }*/

            //drive code

            x = Smoothing.SmoothPerpendicularInput(Math.copySign( gamepad1.left_stick_x, gamepad1.left_stick_x));

            y = -Smoothing.SmoothParallelInput(Math.copySign( gamepad1.left_stick_y, gamepad1.left_stick_y));
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
                    headingsetpoint += gamepad1.right_stick_x * 2;
                }
            }else if(gamepad1.right_trigger > .1) {
                drivespeed = 1;
                if(!gamepad1.left_bumper){
                    headingsetpoint += gamepad1.right_stick_x * 10;
                }
            }else{
                drivespeed = .7;
                if(!gamepad1.left_bumper){
                    headingsetpoint += gamepad1.right_stick_x * 4;
                }
            }

            HDing.headingP = zP;
            HDing.headingD = zD;


            HDing.HeadingMethod(headingsetpoint, 300, ODO.HeadingDEG, getRuntime());

            z = HDing.headingPower;
            if(z > 1){
                z = 1;
            }else if (z < -1){
                z = -1;
            }

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
            dashboardTelemetry.addData("heading Speed", HDing.headingCurrentSpeed);
            dashboardTelemetry.update();
            telemetry.addData("aligment bar" ,alignmentBarSet);
            telemetry.addData("right encoder RAW", robot.MotorVR.getCurrentPosition());
            telemetry.addData("Left encoder RAW", robot.MotorVL.getCurrentPosition());
            telemetry.addData("back encoder RAW", robot.MotorHL.getCurrentPosition());

            telemetry.addData("intake distance sensor", robot.IntakeV3.getDistance(DistanceUnit.INCH));
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
            dashboardTelemetry.addData("liftpos", liftcurrentpos);
            dashboardTelemetry.addData("lift speed", lift.liftSpeed);
            dashboardTelemetry.addData("lift set", liftset);
            dashboardTelemetry.addData("lift power", lift.liftpower);

            telemetry.addData("IMU v", IMU);
            telemetry.addData("liftset", liftset);
            telemetry.addData("lift power", lift.liftpower);
            telemetry.addData("liftpos", liftcurrentpos);


            telemetry.update();

        }
    }
}
