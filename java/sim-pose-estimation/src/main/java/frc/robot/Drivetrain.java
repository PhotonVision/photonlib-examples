// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;


public class Drivetrain {


  PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
  PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
  PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
  PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

  SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftLeader, m_leftFollower);
  SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader, m_rightFollower);

  Encoder m_leftEncoder = new Encoder(Constants.kDtLeftEncoderPinA, Constants.kDtLeftEncoderPinB);
  Encoder m_rightEncoder = new Encoder(Constants.kDtRightEncoderPinA, Constants.kDtRightEncoderPinB);

  DrivetrainPoseEstimator poseEst = new DrivetrainPoseEstimator();


  DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.kTrackWidth);

  // Gains are for example purposes only - must be determined for your own
  // robot!
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  public Drivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightGroup.setInverted(true);

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

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    poseEst.resetToPose(pose);
  }

  public Pose2d getCtrlsPoseEstimate() {
    return poseEst.getPoseEst();
  }

  public void periodic() {
    poseEst.update(new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()),
    m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

}
