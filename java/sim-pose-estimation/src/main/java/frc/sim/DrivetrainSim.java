package frc.sim;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants;


/**
 * Implementation of a simulation of robot physics, sensors, motor controllers
 * Includes a Simulated PhotonVision system and one vision target.
 * 
 * This class and its methods are only relevant during simulation. While on the real robot, the
 * real motors/sensors/physics are used instead.
 */
public class DrivetrainSim {

  // Simulated Sensors
  AnalogGyroSim m_gyroSim = new AnalogGyroSim(Constants.kGyroPin);
  EncoderSim m_leftEncoderSim = EncoderSim.createForChannel(Constants.kDtLeftEncoderPinA);
  EncoderSim m_rightEncoderSim = EncoderSim.createForChannel(Constants.kDtRightEncoderPinA);
 
  // Simulated Motor Controllers
  PWMSim m_leftLeader = new PWMSim(Constants.kDtLeftLeaderPin);
  PWMSim m_leftFollower = new PWMSim(Constants.kDtLeftFollowerPin);
  PWMSim m_rightLeader = new PWMSim(Constants.kDtRightLeaderPin);
  PWMSim m_rightFollower = new PWMSim(Constants.kDtRightFollowerPin);

  // Simulation Physics
  // Configure these to match your drivetrain's physical dimensions
  // and characterization results.
  LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8, Constants.kTrackWidth, Constants.kWheelRadius, null);

  // Simulated Vision System.
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

  /**
   * Perform all periodic drivetrain simulation related tasks to advance our simulation 
   * of robot physics forward by a single 20ms step.
   */
  public void update(){
    // Roughly model the effect of leader and follower motor pushing on the same gearbox.
    // Note if the software is incorrect and drives them against each other, speed will be 
    // roughly matching the physical situation, but current draw will _not_ be accurate.
    double leftMotorCmd = (m_leftLeader.getSpeed() + m_leftFollower.getSpeed())/2.0;
    double rightMotorCmd = (m_rightLeader.getSpeed() + m_rightFollower.getSpeed())/2.0;


    // Update the physics simulation
    m_drivetrainSimulator.setInputs(
        leftMotorCmd * RobotController.getInputVoltage(),
        -rightMotorCmd * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    // Update our sensors based on the simulated motion of the robot
    m_leftEncoderSim.setDistance((m_drivetrainSimulator.getLeftPositionMeters()));
    m_leftEncoderSim.setRate((m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    m_rightEncoderSim.setDistance((m_drivetrainSimulator.getRightPositionMeters()));
    m_rightEncoderSim.setRate((m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // Gyros have an inverted reference frame for angles, so multiply by -1.0;

    // Update PhotonVision based on our new robot position.
    simVision.processFrame(m_drivetrainSimulator.getPose());

  }


  /**
   * Resets the simulation back to a pre-defined pose
   * Useful to simulate the action of placing the robot onto a specific
   * spot in the field (IE, at the start of each match).
   * @param pose
   */
  public void resetPose(Pose2d pose){
    m_drivetrainSimulator.setPose(pose);
  }

  /**
   * @return The simulated robot's position, in meters.
   */
  public Pose2d getCurPose(){
      return m_drivetrainSimulator.getPose();
  }
  
    
}
