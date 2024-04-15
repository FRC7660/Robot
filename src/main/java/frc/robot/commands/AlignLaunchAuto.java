// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Launch.LaunchPreset;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignLaunchAuto extends Command {
  private final SwerveSubsystem swerve;
  private final Launcher launcher;
  private final LaunchPreset launchPreset;
  private final double launchTime;
  private final Timer timer;
  private final Index index;

  /** Creates a new AlignLaunchAuto. */
  public AlignLaunchAuto(
      SwerveSubsystem swerve,
      Launcher launcher,
      Index index,
      LaunchPreset launchPreset,
      double launchTime) {
    this.swerve = swerve;
    this.launcher = launcher;
    this.index = index;
    this.launchPreset = launchPreset;
    this.launchTime = launchTime;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher, index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    launcher.setLaunchPreset(launchPreset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.setPresetVelo();
    if (swerve.align()) {
      if (launcher.isAtTargetVelo()) {
        index.feed();
        timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(launchTime);
  }
}
