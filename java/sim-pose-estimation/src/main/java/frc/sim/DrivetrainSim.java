package frc.sim;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants;


public class DrivetrainSim {

  // Simulation classes help us simulate our robot
  AnalogGyroSim m_gyroSim = new AnalogGyroSim(Constants.kGyroPin);
  EncoderSim m_leftEncoderSim = EncoderSim.createForChannel(Constants.kDtLeftEncoderPinA);
  EncoderSim m_rightEncoderSim = EncoderSim.createForChannel(Constants.kDtRightEncoderPinA);
  LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8, Constants.kTrackWidth, Constants.kWheelRadius, null);

  // Configure these to match your PhotonVision Camera,
  // pipeline, and LED setup. 
  double camDiagFOV = 75.0; // degrees
  double camPitch = 0.0;     // degrees
  double camHeightOffGround = 0.85; // meters
  double maxLEDRange = 20;          // meters
  int camResolutionWidth = 640;     // pixels
  int camResolutionHeight = 480;    // pixels
  double minTargetArea = 10;        // square pixels
      
  SimVisionSystem simVision  = new SimVisionSystem(Constants.kCamName,
                                                    camDiagFOV,
                                                    camPitch,
                                                    Constants.kCameraToRobot,
                                                    camHeightOffGround,
                                                    maxLEDRange,
                                                    camResolutionWidth,
                                                    camResolutionHeight,
                                                    minTargetArea);



  public DrivetrainSim(){
    simVision.addSimVisionTarget(Constants.kFarTarget);
  }

  public void update(){
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftLeader.get() * RobotController.getInputVoltage(),
        -m_rightLeader.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    simVision.processFrame(m_drivetrainSimulator.getPose());

  }

  public void resetPose(Pose2d pose){
    m_leftEncoderSim.resetData();
    m_rightEncoderSim.resetData();
    m_drivetrainSimulator.setPose(pose);
  }
  
    
}
