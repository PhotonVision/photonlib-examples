package frc.robot;

import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Holding class for all physical constants that must be used throughout the codebase.
 * These values should be set by one of a few methods:
 * 1) Talk to your mechanical and electrical teams and determine how the physical robot is being built and configured.
 * 2) Read the game manual and look at the field drawings
 * 3) Match with how your vision coprocessor is configured.
 */
public class Constants {

  //////////////////////////////////////////////////////////////////
  // Drivetrain Physical
  //////////////////////////////////////////////////////////////////
  public static final double kMaxSpeed = 3.0;   // 3 meters per second.
  public static final double kMaxAngularSpeed = Math.PI;   // 1/2 rotation per second.

  public static final double kTrackWidth  = 0.381 * 2;
  public static final double kWheelRadius = 0.0508;
  public static final int kEncoderResolution = -4096;

  //////////////////////////////////////////////////////////////////
  // Electrical IO
  //////////////////////////////////////////////////////////////////
  public static final int kGyroPin = 0;

  public static final int kDtLeftEncoderPinA  = 0;
  public static final int kDtLeftEncoderPinB  = 1;
  public static final int kDtRightEncoderPinA = 2;
  public static final int kDtRightEncoderPinB = 3;

  public static final int kDtLeftLeaderPin    = 1;
  public static final int kDtLeftFollowerPin  = 2;
  public static final int kDtRightLeaderPin   = 3;
  public static final int kDtRightFollowerPin = 4;


  //////////////////////////////////////////////////////////////////
  // Vision Processing 
  //////////////////////////////////////////////////////////////////
  // Name configured in the PhotonVision GUI for the camera
  public static final String kCamName = "mainCam"; 

  // Physical location of the camera on the robot, relative to the center of the robot.
  public static final Transform2d kCameraToRobot = new Transform2d(new Translation2d(-0.25, 0), // in meters 
                                                                  new Rotation2d()); 

  // Definition for the opposite-alliance high goal in 2020 & 2021
  public static final double targetHeightAboveGround = 2.3; // meters
  public static final double targetWidth  = 0.54;           // meters
  public static final double targetHeight = 0.25;           // meters
  public static final Pose2d kFarTargetPose = new Pose2d(new Translation2d(Units.feetToMeters(54),Units.feetToMeters(10)), 
                                                        new Rotation2d(0.0)); 
  public static final SimVisionTarget kFarTarget = new SimVisionTarget(kFarTargetPose, targetHeightAboveGround, targetWidth, targetHeight);

    
}
