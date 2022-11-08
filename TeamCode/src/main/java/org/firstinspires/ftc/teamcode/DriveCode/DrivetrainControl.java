package org.firstinspires.ftc.teamcode.DriveCode;

public class DrivetrainControl {

    HeadingControl Heading = new HeadingControl();
    DirectionCalc Direction = new DirectionCalc();
    SpeedClass Speed = new SpeedClass();

    public double FinalX = 0;
    public double FinalY = 0;

    public double RLDIR, RRDIR, LRDIR, LLDIR;

    double MaxMotor = 0;

    double MotorSpeed = 0;



    public void Drivetrain(double paraSet, double perpSet, double parastart, double perpstart, double speedSet, double rampDownDist, double headingSet,
                           double headingSpeed, double currentpara, double currentperp, double currentheading, double time){

        Direction.DirectionMethod(paraSet, perpSet, parastart, perpstart, currentpara, currentperp);
        Speed.SpeedCalc(speedSet, 1 ,rampDownDist, Direction.distanceFrom,currentpara, currentperp, time);
        Heading.HeadingMethod(headingSet, headingSpeed, currentheading, time);

        FinalY = Speed.speedPower * Math.sin(Math.toRadians(Direction.directionVector));
        FinalX = Speed.speedPower * Math.cos(Math.toRadians(Direction.directionVector));

        LLDIR = -FinalY + Heading.headingPower;
        LRDIR = FinalY + Heading.headingPower;
        RLDIR = -FinalX + Heading.headingPower;
        RRDIR = FinalX + Heading.headingPower;

        MaxMotor = Math.max(Math.max(LLDIR, LRDIR), Math.max(RLDIR, RRDIR));

        LLDIR = LLDIR/MaxMotor;
        LRDIR = LRDIR/MaxMotor;
        RLDIR = RLDIR/MaxMotor;
        RRDIR = RRDIR/MaxMotor;

        MotorSpeed = Speed.speedPower + Heading.headingPower;

        if(MotorSpeed > 1){
            MotorSpeed = 1;
        }else if(MotorSpeed < -1){
            MotorSpeed = -1;
        }

        LLDIR = LLDIR * MotorSpeed;
        LRDIR = LRDIR * MotorSpeed;
        RLDIR = RLDIR * MotorSpeed;
        RRDIR = RRDIR * MotorSpeed;



    }
}
