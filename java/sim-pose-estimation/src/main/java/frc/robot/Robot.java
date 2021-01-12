// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.sim.DrivetrainSim;

public class Robot extends TimedRobot {

  AutoController autoCtrl = new AutoController();
  Drivetrain dt = new Drivetrain();
  OperatorInterface opInf = new OperatorInterface();

  DrivetrainSim dtSim = new DrivetrainSim();

  PoseTelemetry pt = new PoseTelemetry();

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    setNetworkTablesFlushEnabled(true);
  }

  @Override
  public void autonomousInit() {
    resetOdometery();
    autoCtrl.startPath();
  }

  @Override
  public void autonomousPeriodic() {
    ChassisSpeeds speeds = autoCtrl.getCurMotorCmds(dt.getCtrlsPoseEstimate());
    dt.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    pt.setDesiredPose(autoCtrl.getCurPose2d());
  }

  @Override
  public void teleopPeriodic() {
    dt.drive(opInf.getFwdRevSpdCmd(), opInf.getRotateSpdCmd());
  }

  @Override
  public void robotPeriodic() {
    pt.setEstimatedPose(dt.getCtrlsPoseEstimate());
    pt.update();
  }

  @Override
  public void disabledPeriodic() {
    dt.drive(0, 0);
  }

  @Override
  public void simulationPeriodic() {
    if (opInf.getSimKickCmd()) {
      dtSim.applyKick();
    }
    dtSim.update();
    pt.setActualPose(dtSim.getCurPose());
  }

  private void resetOdometery() {
    Pose2d startPose = autoCtrl.getInitialPose();
    dtSim.resetPose(startPose);
    dt.resetOdometry(startPose);
  }
}
