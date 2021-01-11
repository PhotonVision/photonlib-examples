package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;

public class DrivetrainPoseEstimator {

    private final AnalogGyro m_gyro = new AnalogGyro(Constants.kGyroPin);

    Matrix<N5, N1>  stateStdDevs  = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05); 
    Matrix<N3, N1>  localMeasurementStdDevs  = VecBuilder.fill(0.01,0.01,Units.degreesToRadians(0.1));
    Matrix<N3, N1>  visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  
    private final DifferentialDrivePoseEstimator m_poseEstimator = 
    new DifferentialDrivePoseEstimator(m_gyro.getRotation2d(), new Pose2d(), 
    stateStdDevs, 
    localMeasurementStdDevs, 
    visionMeasurementStdDevs);

    private PhotonCamera cam = new PhotonCamera(Constants.kCamName);

    public DrivetrainPoseEstimator(){

    }

    public void update(DifferentialDriveWheelSpeeds actWheelSpeeds, double leftDist, double rightDist){
        m_poseEstimator.update( m_gyro.getRotation2d(),
        actWheelSpeeds, 
        leftDist, rightDist);

        var res = cam.getLatestResult();   
        if(res.hasTargets()){
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
            Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
            Pose2d camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            m_poseEstimator.addVisionMeasurement( camPose.transformBy(Constants.kCameraToRobot), imageCaptureTime);
        }
    }

    public void resetToPose(Pose2d pose){
        m_poseEstimator.resetPosition(pose, m_gyro.getRotation2d());
    }

    public Pose2d getPoseEst() {
        return m_poseEstimator.getEstimatedPosition();
      }
    
}
