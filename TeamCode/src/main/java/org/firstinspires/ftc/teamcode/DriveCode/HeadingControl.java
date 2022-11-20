package org.firstinspires.ftc.teamcode.DriveCode;

public class HeadingControl {
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

        headingError = headingsetpoint - currentheading;

       // if((headingsetpoint - currentheading) < 180)
        deltaHeading = currentheading - lastHeading;

        if(headingsetpoint > currentheading){
            directionHeading = 1;
        }else if(headingsetpoint < currentheading){
            directionHeading = -1;
        }

        speedSetHeading = Math.copySign(headingspeedsetpoint, directionHeading);

        if(Math.abs(headingsetpoint - currentheading) < 15){
            speedSetHeading = speedSetHeading * (Math.abs(headingsetpoint - currentheading)/ 15);

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
