package org.firstinspires.ftc.teamcode.DriveCode;

import org.firstinspires.ftc.teamcode.LiftClasses.LiftControl;

public class HeadingControl {
    LiftControl Lift = new LiftControl();
    double lastHeading = 0;
    double deltaHeading = 0;
    double directionHeading = 1;
    double speedSetHeading = 0;
    public double headingCurrentSpeed = 0;
    double lastTime = 0;
    public double headingPower = 0;
    public double headingP = .00004;
    public double inmotionprofileheading = 0;
    double headingSpeedError = 0;
    double lastHeadingSpeedError = 0;
    public double headingD = 0.002;
    public double headingError = 0;

    public void HeadingMethod(double headingsetpoint, double headingspeedsetpoint, double currentheading, double time){
     /*   if(!Lift.isJake2){
            headingP = .00004;
            headingD = 0.002;
        }else{
            headingP = .00045;
            headingD = 0.002;
        }*/

        headingError = headingsetpoint - currentheading;


        deltaHeading = currentheading - lastHeading;

        if(headingsetpoint > currentheading){
            directionHeading = 1;
        }else if(headingsetpoint < currentheading){
            directionHeading = -1;
        }

        speedSetHeading = Math.copySign(headingspeedsetpoint, directionHeading);

        if(Math.abs(headingsetpoint - currentheading) < 30){
            speedSetHeading = speedSetHeading * (Math.abs(headingsetpoint - currentheading)/ 30);

        }else{
            inmotionprofileheading = 0;
        }

        headingCurrentSpeed = deltaHeading/(time - lastTime);

        headingSpeedError = speedSetHeading - headingCurrentSpeed;

        headingPower += (headingSpeedError* headingP) + ((headingSpeedError - lastHeadingSpeedError) * headingD);

        if(headingPower > 1){
            headingPower = 1;
        }else if(headingPower < -1){
            headingPower = -1;
        }

        lastHeading = currentheading;
        lastHeadingSpeedError = headingSpeedError;
        lastTime = time;
    }
}


