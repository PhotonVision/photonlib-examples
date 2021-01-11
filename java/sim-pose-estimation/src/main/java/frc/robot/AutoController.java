package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class AutoController {
    
    private Trajectory m_trajectory;

    private RamseteController m_ramsete = new RamseteController();

    private Timer m_timer = new Timer();

    boolean isRunning = false;


    public AutoController(){

        m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));
    }


    Pose2d getInitialPose(){
        return m_trajectory.getInitialPose();
    }

    void startPath(){
        m_timer.reset();
        m_timer.start();
        isRunning = true;

    }

    void stopPath(){
        isRunning = false;
        m_timer.reset();
    }

    ChassisSpeeds getCurMotorCmds( Pose2d curEstPose ){
        Trajectory.State reference ;
        
        if(isRunning){
            double elapsed = m_timer.get();
            reference = m_trajectory.sample(elapsed);
        } else {
            reference = new Trajectory.State();
        }

        return m_ramsete.calculate(curEstPose, reference);
    }

}
