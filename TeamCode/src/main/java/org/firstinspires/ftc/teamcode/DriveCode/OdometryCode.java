package org.firstinspires.ftc.teamcode.DriveCode;

public class OdometryCode {
    //test
    
    public double ParaLeftEncoder = 0;
    public double ParaRightEncoder = 0;
    public double PerpEncoder = 0;
    double LastParaLeftEncoder = 0;
    double LastParaRightEncoder = 0;
    double LastPerpEncoder = 0;
    double ChangeParaLeftEncoder;
    double ChangeParaRightEncoder;
    double ChangePerpEncoder;
    double ChangeParaCombined = 0;
    double ChangePerpCombined = 0;
    static double TicksToInches = 236.5;
    public double TrackWidth = 12.5;
    public double PerpOffset = 6.25;
    double ChangeHeading = 0;
    public double HeadingRAD = 0;
    public double HeadingDEG = 0;
    double ChangePerp = 0;
    double ChangePara = 0;
    public double ParaDist = 0;
    public double PerpDist = 0;
    
    public void OdoCalc(double RAWparallelLeftEncoder, double RAWPerpendicularEncoder, double RAWparallelRightEncoder){


        
        ParaLeftEncoder = RAWparallelLeftEncoder/TicksToInches;
        ParaRightEncoder = -RAWparallelRightEncoder/TicksToInches;
        PerpEncoder = RAWPerpendicularEncoder/TicksToInches;
        
        ChangeParaLeftEncoder = LastParaLeftEncoder - ParaLeftEncoder;
        ChangeParaRightEncoder = LastParaRightEncoder - ParaRightEncoder;
        ChangePerpEncoder = LastPerpEncoder - PerpEncoder;


        ChangeHeading = (ChangeParaRightEncoder - ChangeParaLeftEncoder)/TrackWidth;
        ChangeParaCombined = (ChangeParaRightEncoder + ChangeParaLeftEncoder)/2;
        ChangePerpCombined = ChangePerpEncoder - PerpOffset * ChangeHeading;


        HeadingRAD += ChangeHeading;
        HeadingDEG = Math.toDegrees(HeadingRAD);

        ChangePara = ChangeParaCombined * Math.cos(HeadingRAD) - ChangePerpCombined * Math.sin(HeadingRAD);
        ChangePerp = ChangeParaCombined * Math.sin(HeadingRAD) + ChangePerpCombined * Math.cos(HeadingRAD);

        ParaDist -= ChangePara;
        PerpDist -= ChangePerp;

        
        
        
        LastParaLeftEncoder = ParaLeftEncoder;
        LastParaRightEncoder = ParaRightEncoder;
        LastPerpEncoder = PerpEncoder;
        
        
    }
}
