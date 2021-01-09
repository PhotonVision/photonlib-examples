/**
 * Copyright (C) 2018-2020 Photon Vision.
 * <p>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * <p>
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * <p>
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // Change this to match
    PhotonCamera camera = new PhotonCamera("photonvision");

    XboxController xboxController;

    // Drive motors
    PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    Encoder leftEncoder = new Encoder(0, 1);
    Encoder rightEncoder = new Encoder(2, 3);
    Gyro gyro = new AnalogGyro(0);

    Field2d field = new Field2d();

    Pose2d fieldToTarget = new Pose2d(Units.feetToMeters(54), Units.inchesToMeters(94.66), new Rotation2d());

    // Represents a camera mounted 6in behind the robot's "origin" (usually the center)
    Transform2d cameraToRobot = new Transform2d(new Translation2d(Units.inchesToMeters(6), 0), new Rotation2d());

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = 2.45;

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(gyro.getRotation2d(), new Pose2d(), VecBuilder.fill(0.01, 0.01, 0.05, 0.1, 0.1), VecBuilder.fill(0.001, 0.001, 0.05), VecBuilder.fill(0.5, 0.5, 0.5));

    static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    static final double ENCODER_CPR = 4096;
    // Output over input, so greater than one in most cases.
    static final double GEARING = 6;

    @Override
    public void robotInit() {
        leftEncoder.setDistancePerPulse(2.0 * Math.PI * WHEEL_RADIUS / ENCODER_CPR / GEARING);
        rightEncoder.setDistancePerPulse(2.0 * Math.PI * WHEEL_RADIUS / ENCODER_CPR / GEARING);

        xboxController = new XboxController(0);
    }

    @Override
    public void robotPeriodic() {
        estimator.update(gyro.getRotation2d(), new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate()), leftEncoder.getDistance(), rightEncoder.getDistance());

        var result = camera.getLatestResult();
        if (result.hasTargets()) {

            var fieldToRobot = PhotonUtils.estimateFieldToRobot(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                TARGET_HEIGHT_METERS,
                Rotation2d.fromDegrees(-result.getBestTarget().getYaw()),
                gyro.getRotation2d(),
                fieldToTarget,
                cameraToRobot
            );

            // We'll add this pose measurement to our pose estimator
            estimator.addVisionMeasurement(fieldToRobot, Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000.0);
        }

        field.setRobotPose(estimator.getEstimatedPosition());
    }
}
