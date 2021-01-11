package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseTelemetry {

    Field2d m_fieldPoseEst = new Field2d();
    Field2d m_fieldPoseAct = new Field2d();
    Field2d m_fieldPoseDes = new Field2d();


    public PoseTelemetry(){
        SmartDashboard.putData("Field_EstPose", m_fieldPoseEst);
        SmartDashboard.putData("Field_ActPose", m_fieldPoseAct);
        SmartDashboard.putData("Field_DesPose", m_fieldPoseDes);
    }

    public void setActualPose(Pose2d act_in){
        m_fieldPoseAct.setRobotPose(act_in);
    }

    public void setDesiredPose(Pose2d des_in){
        m_fieldPoseDes.setRobotPose(des_in);
    }

    public void setEstimatedPose(Pose2d est_in){
        m_fieldPoseEst.setRobotPose(est_in);
    }
    
}
