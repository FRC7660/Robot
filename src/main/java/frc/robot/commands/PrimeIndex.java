// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Transfer;

public class PrimeIndex extends Command {
  private final Index index;
  private final Transfer transfer;
  private boolean inUpper;
  private boolean wasRunning;

  /**
   * Creates a new PrimeIndex.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PrimeIndex(Index index, Transfer transfer) {
    this.index = index;
    this.transfer = transfer;
    inUpper = false;
    wasRunning = index.isRunning();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Index is initializing");
    index.start();
    transfer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!inUpper && index.isInUpper()) {
      inUpper = true;
      transfer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stop();
    transfer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index.isPrimed();
  }
}
