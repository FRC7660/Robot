// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeAndIndex extends Command {
  /** Creates a new IntakeAndIndex. */
  private final Intake intake;

  private final Transfer transfer;
  private final Index index;
  private boolean lowerHit;

  public IntakeAndIndex(Intake intake, Index index, Transfer transfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.index = index;
    this.transfer = transfer;
    lowerHit = false;

    addRequirements(intake, index, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.start();
    index.start();
    transfer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!lowerHit) {
      lowerHit = index.getLowerSensorHit();
      if (lowerHit) {
        intake.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    index.stop();
    transfer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index.getUpperSensorHit();
  }
}
