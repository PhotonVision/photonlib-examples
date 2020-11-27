/**
 * Copyright (C) 2018-2020 Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  PIDController controller = new PIDController(.1, 0, 0);

  XboxController xboxController = new XboxController(0);

  // Drive motors
  PWMVictorSPX leftMotor = new PWMVictorSPX(0);
  PWMVictorSPX rightMotor = new PWMVictorSPX(1);
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  @Override
  public void teleopPeriodic() {
    double forwardSpeed = xboxController.getY(GenericHID.Hand.kRight);
    double rotationSpeed;

    if (xboxController.getAButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        // Rotation speed is the output of the PID controller
        rotationSpeed = controller.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
      }
    } else {
      // Manual Driver Mode
      rotationSpeed = xboxController.getX(GenericHID.Hand.kLeft);
    }

    // Use our forward/turn speeds to control the drivetrain
    drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}
