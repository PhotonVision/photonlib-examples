// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;

@SuppressWarnings("PMD.TooManyFields")
public class Drivetrain {
  // 3 meters per second.
  public static final double kMaxSpeed = 3.0;
  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = Math.PI;

  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;

  private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
  private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
  private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
  private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

  private final SpeedControllerGroup m_leftGroup =
      new SpeedControllerGroup(m_leftLeader, m_leftFollower);
  private final SpeedControllerGroup m_rightGroup =
      new SpeedControllerGroup(m_rightLeader, m_rightFollower);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  Matrix<N5, N1>  stateStdDevs  = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05); 
  Matrix<N3, N1>  localMeasurementStdDevs  = VecBuilder.fill(0.01,0.01,Units.degreesToRadians(0.1));
  Matrix<N3, N1>  visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

  private final DifferentialDrivePoseEstimator m_poseEstimator = 
      new DifferentialDrivePoseEstimator(m_gyro.getRotation2d(), new Pose2d(), 
      stateStdDevs, 
      localMeasurementStdDevs, 
      visionMeasurementStdDevs);

  private PhotonCamera cam = new PhotonCamera("mainCam");


  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation classes help us simulate our robot
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator =
  new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

  double camDiagFOV = 75.0; // degrees
  double camPitch = 0.0;     // degrees
  Transform2d cameraToRobot = new Transform2d(); // meters
  double camHeightOffGround = 0.85; // meters
  double maxLEDRange = 20;          // meters
  int camResolutionWidth = 640;     // pixels
  int camResolutionHeight = 480;    // pixels
  double minTargetArea = 10;        // square pixels
      
  private final SimVisionSystem simVision  = new SimVisionSystem("mainCam",
                                                              camDiagFOV,
                                                              camPitch,
                                                              cameraToRobot,
                                                              camHeightOffGround,
                                                              maxLEDRange,
                                                              camResolutionWidth,
                                                              camResolutionHeight,
                                                              minTargetArea);

  Pose2d targetPose = new Pose2d(new Translation2d(Units.feetToMeters(54),Units.feetToMeters(10)), 
                                 new Rotation2d()); // meters
  double targetHeightAboveGround = 2.3; // meters
  double targetWidth = 0.54;           // meters
  double targetHeight = 0.25;          // meters
  

  public Drivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);

    simVision.addSimVisionTarget( new SimVisionTarget(targetPose,
    targetHeightAboveGround,
    targetWidth,
    targetHeight));
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  public void updateOdometry() {
    m_poseEstimator.update( m_gyro.getRotation2d(),
                            new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()), 
                            m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    var res = cam.getLatestResult();   
    if(res.hasTargets()){
      double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
      Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
      Pose2d camPose = targetPose.transformBy(camToTargetTrans.inverse());
      m_poseEstimator.addVisionMeasurement( camPose.transformBy(cameraToRobot), imageCaptureTime);
    }
  }

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_drivetrainSimulator.setPose(pose);
    m_poseEstimator.resetPosition(pose, m_gyro.getRotation2d());
  }

  public Pose2d getCtrlsPoseEstimate() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void simulationPeriodic() {
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

  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }
}
