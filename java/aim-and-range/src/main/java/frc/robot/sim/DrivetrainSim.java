package frc.robot.sim;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Implementation of a simulation of robot physics, sensors, motor controllers
 * Includes a Simulated PhotonVision system and one vision target.
 * 
 * This class and its methods are only relevant during simulation. While on the
 * real robot, the real motors/sensors/physics are used instead.
 */
public class DrivetrainSim {

    // Simulated Motor Controllers
    PWMSim leftLeader = new PWMSim(0);
    PWMSim rightLeader = new PWMSim(1);

    // Simulation Physics
    // Configure these to match your drivetrain's physical dimensions
    // and characterization results.
    LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(drivetrainSystem, DCMotor.getCIM(2),
            8, Units.feetToMeters(2.0), Units.inchesToMeters(6.0/2.0), null);

    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 150.0; // degrees
    double camPitch = Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees
    double camHeightOffGround = Robot.CAMERA_HEIGHT_METERS; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    SimVisionSystem simVision = new SimVisionSystem("photonvision", camDiagFOV, camPitch, new Transform2d(),
            camHeightOffGround, maxLEDRange, camResolutionWidth, camResolutionHeight, minTargetArea);

    double targetWidth = 0.54; // meters
    double targetHeight = 0.25; // meters
    Pose2d farTargetPose = new Pose2d(
        new Translation2d(Units.feetToMeters(54), Units.feetToMeters(10)), new Rotation2d(0.0));


    Field2d field = new Field2d();

    public DrivetrainSim() {
        simVision.addSimVisionTarget(new SimVisionTarget(farTargetPose, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
        SmartDashboard.putData("Field", field);
    }

    /**
     * Perform all periodic drivetrain simulation related tasks to advance our
     * simulation of robot physics forward by a single 20ms step.
     */
    public void update() {

        double leftMotorCmd = 0;
        double rightMotorCmd = 0;

        if (DriverStation.getInstance().isEnabled() && !RobotController.isBrownedOut()) {
            leftMotorCmd = leftLeader.getSpeed();
            rightMotorCmd = rightLeader.getSpeed();
        }

        drivetrainSimulator.setInputs(leftMotorCmd * RobotController.getInputVoltage(),
                -rightMotorCmd * RobotController.getInputVoltage());
        drivetrainSimulator.update(0.02);

        // Update PhotonVision based on our new robot position.
        simVision.processFrame(drivetrainSimulator.getPose());

        field.setRobotPose(drivetrainSimulator.getPose());
    }

    /**
     * Resets the simulation back to a pre-defined pose Useful to simulate the
     * action of placing the robot onto a specific spot in the field (IE, at the
     * start of each match).
     * 
     * @param pose
     */
    public void resetPose(Pose2d pose) {
        drivetrainSimulator.setPose(pose);
    }

}
