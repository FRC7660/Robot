// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax motorLeft =
      new CANSparkMax(Constants.Intake.leftCANID, CANSparkLowLevel.MotorType.kBrushless);

  private CANSparkMax motorCenter =
      new CANSparkMax(Constants.Intake.centerCANID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorRight =
      new CANSparkMax(Constants.Intake.rightCANID, CANSparkLowLevel.MotorType.kBrushless);

  double speed = Constants.Intake.speed;
  double currentSpeed = 0.0;

  public Intake() {
    motorLeft.setInverted(Constants.Intake.leftInverted);
    motorCenter.setInverted(Constants.Intake.centerInverted);
    motorRight.setInverted(Constants.Intake.rightInverted);
    stop();
  }

  private void set(double power) {
    motorLeft.set(power);
    motorCenter.set(power);
    motorRight.set(power);
    currentSpeed = power;
  }

  public void toggle() {
    if (currentSpeed == 0.0) {
      start();
    } else {
      stop();
    }
  }

  public void start() {
    set(speed);
  }

  public void reverse() {
    set(-speed);
  }

  public void stop() {
    set(0.0);
  }

  public boolean isRunning() {
    return !(motorLeft.get() == 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Running", isRunning());
  }

  public Command reverseIntakeCommand() {
    return this.run(() -> set(-0.8));
  }
}
