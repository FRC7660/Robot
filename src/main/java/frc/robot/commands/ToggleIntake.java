// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class ToggleIntake extends Command {
  private Intake intake;
  private Transfer transfer;

  /** Creates a new ToggleIntake. */
  public ToggleIntake(Intake intake, Transfer transfer) {
    this.intake = intake;
    this.transfer = transfer;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake is initializing");
    intake.toggle();
    if (intake.isRunning()) {
      transfer.start();
    } else {
      transfer.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
