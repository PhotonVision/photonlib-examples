package frc.robot;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;


public class OperatorInterface {
    private XboxController m_controller = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
    public OperatorInterface(){

    }

    public double getFwdRevSpdCmd(){
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        return -m_speedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft)) * Drivetrain.kMaxSpeed;
    }

    public double getRotateSpdCmd(){
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        return -m_rotLimiter.calculate(m_controller.getX(GenericHID.Hand.kRight))
                * Drivetrain.kMaxAngularSpeed;
    }
    
}
