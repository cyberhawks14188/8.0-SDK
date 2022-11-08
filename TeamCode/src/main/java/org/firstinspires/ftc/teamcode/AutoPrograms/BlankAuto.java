package org.firstinspires.ftc.teamcode.AutoPrograms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.DriveCode.DirectionCalc;
import org.firstinspires.ftc.teamcode.DriveCode.DrivetrainControl;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.firstinspires.ftc.teamcode.TestHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@TeleOp

public class BlankAuto extends LinearOpMode {

    TestHardware robot = new TestHardware();
    OdometryCode ODO = new OdometryCode();
    LiftControl Lift = new LiftControl();
    HeadingControl HDing = new HeadingControl();
    Smoothing Smoothing = new Smoothing();
    DrivetrainControl DriveControl = new DrivetrainControl();
    DirectionCalc DirectionCalc = new DirectionCalc();
    SpeedClass SpeedClass = new SpeedClass();

    //variables for the autonomous

    double action = 1;
    boolean oneloop = false;

    double paraSet = 0, perpSet = 0;
    double paraStart = 0, perpStart = 0;
    double speedSet = 15;
    double rampUpDist = 2, rampDownDist = 4;
    double headingSet = 0, headingSpeedSet = 100;
    double liftSet = 0, liftSpeedSet = 1500;



    //AprilTag initialaization here

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;



    @Override

    public void runOpMode() {
        //initializes FTC dahsboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        //initialized the hardware map
        robot.init(hardwareMap);

        //initilaize the camera for the Apriltags
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // while init loop for the apriltags
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.addData("in init", 0);
            telemetry.update();
            sleep(20);
        }

        //start of init
        waitForStart();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory
        }else if(tagOfInterest.id == MIDDLE){
            //trajectory
        }else{
            //trajectory
        }

        //the auto loop
        while (opModeIsActive()) {
            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());

            if(action == 1){
                paraSet = 20;
                perpSet = 0;
                paraStart = 0;
                perpStart = 0;

                if(DirectionCalc.distanceFrom < 1 && oneloop){
                    action = 2;
                    oneloop = false;
                }else{
                    oneloop = true;
                }

            }else if(action == 2){

            }

            Lift.LiftMethod(liftSet, liftSpeedSet, robot.MotorLift.getCurrentPosition(), getRuntime());
            Drivetrain(paraSet, perpSet, paraStart, perpStart, speedSet, rampUpDist, rampDownDist, headingSet, headingSpeedSet, ODO.ParaDist, ODO.PerpDist, ODO.HeadingDEG, getRuntime());
            robot.MotorVL.setPower(LLDIR);
            robot.MotorVR.setPower(LRDIR);
            robot.MotorHL.setPower(RLDIR);
            robot.MotorHR.setPower(RRDIR);

            telemetry.addData("robot speed", SpeedClass.currentSpeed);
            telemetry.addData("FinalX", FinalX);
            telemetry.addData("FinalY", FinalY);
            telemetry.addData("direction vector", DirectionCalc.directionVector);
            telemetry.addData("slope", DirectionCalc.slope);
            telemetry.addData("speed mpower", SpeedClass.speedPower);
            telemetry.addData("dist from", DirectionCalc.distanceFrom);
            telemetry.addData("para current", ODO.ParaDist);
            telemetry.addData("perp current", ODO.PerpDist);
            telemetry.addData("heading", ODO.HeadingDEG);
            telemetry.addData("para dist from", DirectionCalc.paradistfrom);
            telemetry.addData("action", action);
            telemetry.addData("para dist", ODO.ParaDist);
            telemetry.addData("perp dist", ODO.PerpDist);
            telemetry.addData("hasdistfrom", DirectionCalc.hasDistFrom);
            telemetry.addData("Total dist", DirectionCalc.totalDist);

            telemetry.update();

            dashboardTelemetry.addData("direction vector", DirectionCalc.directionVector);
            dashboardTelemetry.addData("distance from", DirectionCalc.distanceFrom);
            dashboardTelemetry.addData("setX", DirectionCalc.setX);
            dashboardTelemetry.addData("setY", DirectionCalc.setY);
            dashboardTelemetry.addData("current Para", ODO.ParaDist);
            dashboardTelemetry.addData("current Perp", ODO.PerpDist);
            dashboardTelemetry.addData("speed", SpeedClass.currentSpeed);
            dashboardTelemetry.addData("speed Set", SpeedClass.speedSetMod);
            dashboardTelemetry.addData("speedpower", SpeedClass.speedPower);
            dashboardTelemetry.addData("combined motor Speed", MotorSpeed);
            dashboardTelemetry.addData("direction vector", DirectionCalc.directionVector);

            dashboardTelemetry.update();


        }


    }
    public double FinalX = 0;
    public double FinalY = 0;

    public double RLDIR, RRDIR, LRDIR, LLDIR;

    double MaxMotor = 0;

    double MotorSpeed = 0;




    public void Drivetrain(double paraSet, double perpSet, double parastart, double perpstart, double speedSet, double rampUpDist ,double rampDownDist, double headingSet,
                           double headingSpeed, double currentpara, double currentperp, double currentheading, double time){

        DirectionCalc.DirectionMethod(paraSet, perpSet, parastart, perpstart, currentpara, currentperp);
        SpeedClass.SpeedCalc(speedSet, rampUpDist,rampDownDist, DirectionCalc.distanceFrom, currentpara, currentperp, time);
        HDing.HeadingMethod(headingSet, headingSpeed, currentheading, time);


        FinalY = 1 * Math.cos(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));
        FinalX = 1 * Math.sin(Math.toRadians(DirectionCalc.directionVector - ODO.HeadingDEG));

        LLDIR = FinalY + HDing.headingPower;
        LRDIR = -FinalY + HDing.headingPower;
        RLDIR = -FinalX + HDing.headingPower;
        RRDIR = FinalX + HDing.headingPower;

        MaxMotor = Math.max(Math.max(Math.abs(LLDIR), Math.abs(LRDIR)), Math.max(Math.abs(RLDIR), Math.abs(RRDIR)));

        LLDIR = LLDIR/MaxMotor;
        LRDIR = LRDIR/MaxMotor;
        RLDIR = RLDIR/MaxMotor;
        RRDIR = RRDIR/MaxMotor;

        MotorSpeed = Math.abs(SpeedClass.speedPower + HDing.headingPower);

        if(MotorSpeed > 1){
            MotorSpeed = 1;
        }else if(MotorSpeed < -1){
            MotorSpeed = -1;
        }


        if(DirectionCalc.distanceFrom < .5 && Math.abs(HDing.headingError) < 2){
            HDing.headingPower = 0;
            SpeedClass.speedPower = 0;
            LLDIR = 0;
            LRDIR = 0;
            RLDIR = 0;
            RRDIR = 0;
        }else{
            LLDIR = LLDIR * MotorSpeed;
            LRDIR = LRDIR * MotorSpeed;
            RLDIR = RLDIR * MotorSpeed;
            RRDIR = RRDIR * MotorSpeed;
        }






    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}


