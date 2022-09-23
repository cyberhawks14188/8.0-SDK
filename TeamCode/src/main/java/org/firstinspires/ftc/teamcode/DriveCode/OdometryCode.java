package org.firstinspires.ftc.teamcode.DriveCode;

public class OdometryCode {
    //test
    
    double ParaLeftEncoder = 0;
    double ParaRightEncoder = 0;
    double PerpEncoder = 0;
    double LastParaLeftEncoder = 0;
    double LastParaRightEncoder = 0;
    double LastPerpEncoder = 0;
    double ChangeParaLeftEncoder;
    double ChangeParaRightEncodr;
    double ChangePerpEncoder;
    
    public void OdoCalc(double RAWparallelLeftEncoder, double RAWparallelRightEncoder, double RAWPerpendicularEncoder){
        
        ParaLeftEncoder = RAWparallelLeftEncoder;
        ParaRightEncoder = RAWparallelRightEncoder;
        PerpEncoder = RAWPerpendicularEncoder;
        
        ChangeParaLeftEncoder = LastParaLeftEncoder - ParaLeftEncoder;
        ChangeParaRightEncoder = LastParaRightEncoder - ParaRightEncoder;
        ChangePerpEncoder = LastPerpEncoder - PerpEncoder;
        
        
        
        LastParaLeftEncoder = ParaleftEncoder;
        LastParaRightEncoder = ParaRightEncoder;
        LastPerpEncoder = PerpEncoder;
        
        
    }
}
