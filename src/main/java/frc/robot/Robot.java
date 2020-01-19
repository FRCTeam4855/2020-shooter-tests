/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  Joystick controller = new Joystick(0);
  Shooter shooter = new Shooter();
  boolean on = false;
  boolean locked = false;
  double setpoint = 0;

  @Override
  public void robotInit() {
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("kP", shooter.kP);
    SmartDashboard.putNumber("kI", shooter.kI);
    SmartDashboard.putNumber("kD", shooter.kD);
    SmartDashboard.putNumber("kF", shooter.kF);
  }

  @Override
  public void teleopPeriodic() {
    shooter.kP = SmartDashboard.getNumber("kP", 0);
    shooter.kI = SmartDashboard.getNumber("kI", 0);
    shooter.kD = SmartDashboard.getNumber("kD", 0);
    shooter.kF = SmartDashboard.getNumber("kF", 0);

    shooter.PID.setP(shooter.kP);
    shooter.PID.setI(shooter.kI);
    shooter.PID.setD(shooter.kD);
    shooter.PID.setFF(shooter.kF);

    if (controller.getRawButtonPressed(1)) {
      if (on) {
        shooter.killFlywheel();
        on = false;
      } else {
        on = true;
      }
    }
    if (controller.getRawButtonPressed(2)) {
      setpoint = 1000;
    }
    if (controller.getRawButtonPressed(3)) {
      setpoint = 2000;
    }
    if (controller.getRawButtonPressed(4)) {
      setpoint = 3400;
    }
    if (on) {
      if (setpoint > 0) locked = shooter.setFlywheelSpeed(setpoint);
      setpoint += Math.signum(controller.getRawAxis(1)) * Math.max(0, Math.abs(controller.getRawAxis(1)) - .1) * 5;
    } else locked = false;

    SmartDashboard.putBoolean("hood on", on);
    SmartDashboard.putBoolean("ready to fire", locked);
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("hood velocity", shooter.encoder.getVelocity());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
