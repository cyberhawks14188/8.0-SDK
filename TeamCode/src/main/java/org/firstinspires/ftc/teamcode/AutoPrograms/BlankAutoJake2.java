package org.firstinspires.ftc.teamcode.AutoPrograms;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.DriveCode.DirectionCalc;
import org.firstinspires.ftc.teamcode.DriveCode.DrivetrainControl;
import org.firstinspires.ftc.teamcode.DriveCode.HeadingControl;
import org.firstinspires.ftc.teamcode.DriveCode.OdometryCode;
import org.firstinspires.ftc.teamcode.DriveCode.Smoothing;
import org.firstinspires.ftc.teamcode.DriveCode.SpeedClass;
import org.firstinspires.ftc.teamcode.Jake_2_Hardware;
import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous

public class BlankAutoJake2 extends LinearOpMode {

    Jake_2_Hardware robot = new Jake_2_Hardware();
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
    boolean vuforiatrigger = false;
    double desiredVuforiaAngle = -32;
    double vuforiaOffset = -100000000;



    //AprilTag initialaization here

    OpenCvCamera APRILcamera;
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

    private static final String VUFORIA_KEY =
            " AZickLn/////AAABmRdNRU8Vt0+EsSkecZ/dEtdwfmReQRmGjONFJw9IrZwj83V0JqVOw7lVMu8esNz/c2srCeQNiZSotXn5mKGHThTl4m0nN9xTmOVBgIkUOrtkA1rGeUkBw0dPy5AD8pk5L4Mv2yikiYUEXDVsPvVYjsp9p2+SHZNPXSBRL8OUPsUa+DpTQnpdRgtca4ZmRFGwUsfqkj/2pTz3/aS8KpFzZ6mjMVKJbJwiZnMhND5Bhy600+NNUNiTka0g6E+9lDEBQI5H0XVkEGCjHIFA0F8Z7L4iIZhotBPNB8kx3ep3MSRQSGg/yrzNIM4av2BqM2JVohuQFh2uSWyDJdgEwxtZ6drh3YZIa12CsuSHNkgaas2k ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;



    @Override

    public void runOpMode() {
        //initializes FTC dahsboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        //initialized the hardware map
        robot.init(hardwareMap);

        //Vuforia Targets here
        webcamName = hardwareMap.get(WebcamName.class, "LeftFacingCamera");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);


        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }


        targets.activate();
        while (vuforiatrigger == false && !isStopRequested()) {
            if(gamepad1.back){
                vuforiatrigger = true;
            }

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                vuforiaOffset = desiredVuforiaAngle - rotation.secondAngle;
                telemetry.addData("VuforiaOffset", vuforiaOffset);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        if(vuforiaOffset == -100000000){
            vuforiaOffset = 0;
        }

        // Disable Tracking when we are done;
        targets.deactivate();
        vuforia.close();

        telemetry.addLine("initializing apriltags");
        telemetry.update();








        //initilaize the camera for the Apriltags
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        APRILcamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "ZoomCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        telemetry.addLine("april");
        telemetry.update();

        APRILcamera.setPipeline(aprilTagDetectionPipeline);
        APRILcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                APRILcamera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("apriltags");
        telemetry.update();

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


        //the auto loop
        ODO.HeadingDEG = vuforiaOffset;
        while (opModeIsActive()) {
            ODO.OdoCalc(robot.MotorVL.getCurrentPosition(), robot.MotorHL.getCurrentPosition(), robot.MotorVR.getCurrentPosition());

            if(action == 1){
                if(tagOfInterest.id == 1){
                    perpSet = -20;
                }else if(tagOfInterest.id == 2){
                    perpSet = 0;
                }else{
                    perpSet = 20;
                }
                paraSet = 20;
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

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

}


