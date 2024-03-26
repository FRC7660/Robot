// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeSpit extends Command {
  /** Creates a new IntakeSpit. */
  private Intake intake;

  private Transfer transfer;

  public IntakeSpit(Intake intake, Transfer transfer) {
    this.intake = intake;
    this.transfer = transfer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.start();
    intake.inAndOut();
    transfer.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    transfer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
