// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;

public class PassiveLaunchSpin extends Command {
  private Launcher m_launcher;
  private Index m_index;

  /** Creates a new PassiveLaunchSpin. */
  public PassiveLaunchSpin(Launcher launcher, Index index) {
    m_launcher = launcher;
    m_index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_index.getUpperSensorHit()) {
      m_launcher.setPresetVelo();
    } else {
      m_launcher.setLaunchVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
