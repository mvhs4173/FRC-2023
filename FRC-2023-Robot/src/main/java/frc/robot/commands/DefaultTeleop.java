// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Motor;
import frc.robot.commands.DefaultTeleop;

public class DefaultTeleop extends CommandBase {
  private final XboxController m_controller;
  private final Motor m_motor;
  /** Creates a new DefaultTeleop. */
  public DefaultTeleop(Motor motor, XboxController controller) {
    m_motor = motor;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motor.beStill(); //sets the motor speed to 0, method is in Motor.java
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = m_controller.getLeftY();
    SmartDashboard.putNumber("Left Y", y);
    if (Math.abs(y) <= 0.025) {
      m_motor.beStill();
    }
    else {
      m_motor.set(y);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motor.beStill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
