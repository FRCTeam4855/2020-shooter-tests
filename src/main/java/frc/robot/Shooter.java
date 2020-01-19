// Shooter class

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    // Define instance variables
    private double velocitySetpoint = 0;        // the setpoint to set the shooter to
    private int speedUpTime = -1;               // the amount of program ticks remaining for the shooter to be fed by percentage input
    private final int MAX_SPEEDUP_TICKS = 100;  // the flywheel will have 100 ticks to increase the speed to roughly where it needs to be via percent output
    private final int ACCEPTABLE_ERROR = 40;    // the acceptable error to reasonably state that the shooter is ready to shoot
    private enum Phase {
        OFF, SPEED_UP, LOCK_IN
    }
    private Phase currentPhase = Phase.OFF;
    double kP = .00044;
    double kI = 0;
    double kD = .0034;
    double kF = .000173;

    // Define hardware
    CANSparkMax flywheel; 
    CANPIDController PID;
    CANEncoder encoder;

    /**
     * Constructs the Shooter class.
     */
    public Shooter() {
        flywheel = new CANSparkMax(3, MotorType.kBrushless);
        encoder = flywheel.getEncoder();
        PID = flywheel.getPIDController();
        PID.setOutputRange(-.85, .85);
        PID.setP(kP);
        PID.setI(kI);
        PID.setD(kD);
        PID.setFF(kF);
    }

    /**
     * Sets the flywheel to a specified velocity. Returns false until the shooter has reached the acceptable speed
     * @param setpoint the desired velocity to set the flywheel to
     * @return true or false depending on the speed has been locked in
     */
    public boolean setFlywheelSpeed(double setpoint) {
        velocitySetpoint = setpoint;
        if (currentPhase == Phase.OFF) {
            currentPhase = Phase.SPEED_UP;
            speedUpTime = MAX_SPEEDUP_TICKS;
        }
        if (speedUpTime > -1 && currentPhase == Phase.SPEED_UP) {
            double percentOutput = -((setpoint / 5100) - .025);
            flywheel.set(percentOutput);
            speedUpTime --;
            if (speedUpTime <= 0) {
                speedUpTime = -1;
                currentPhase = Phase.LOCK_IN;
            }
        }
        if (currentPhase == Phase.LOCK_IN) {
            PID.setReference(-setpoint, ControlType.kVelocity);
            if (Math.abs(setpoint + encoder.getVelocity()) < ACCEPTABLE_ERROR) return true; else return false;
            
        }
        return false;
    }

    public void killFlywheel() {
        currentPhase = Phase.OFF;
        speedUpTime = -1;
        flywheel.set(0);
    }
}