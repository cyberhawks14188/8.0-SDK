package org.firstinspires.ftc.teamcode.LiftClasses;

public class LiftControl {
    public double liftpower = 0;
    double currentTime = 0, lastTime = 0;
    double liftPos = 0, lastliftpos = 0, liftSet;
    double deltaLiftPos = 0;
    double liftDirection = 1;
    double liftSpeedSet = 0;
    double liftSpeed, lastLiftSpeed, liftSpeedDifference = 0;
    public double liftP = .0001;

    public void LiftMethod(double liftset, double liftspeedset, double liftcurrentpos, double time){
        currentTime = time;
        liftPos = liftcurrentpos;

        if(liftset > 1490){
            liftSet = 1490;
        }else if(liftset < 0){
            liftSet = 0;
        }else{
            liftSet = liftset;
        }

        deltaLiftPos = liftPos - lastliftpos;

        if(liftset > liftPos){
            liftDirection = 1;
        }else if(liftset < liftPos){
            liftDirection = -1;
        }

        liftSpeedSet = Math.copySign(liftspeedset, liftDirection);

        if(Math.abs(liftcurrentpos - liftSet) < 200){
            liftSpeedSet = liftSpeedSet * ((Math.abs(liftcurrentpos - liftSet))/200);
        }

        liftSpeed = deltaLiftPos/(currentTime-lastTime);

        liftSpeedDifference = liftSpeedSet - liftSpeed;

        liftpower += (liftSpeedDifference * liftP);

        if(liftpower > 1){
            liftpower = 1;
        }else if(liftpower < -1){
            liftpower = -1;
        }

        lastLiftSpeed = liftSpeed;
        lastliftpos = liftPos;
        lastTime = currentTime;

    }
}
