// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private TalonFX motorRight = new TalonFX(Constants.Intake.rightCANID);

  private SlewRateLimiter limiter = new SlewRateLimiter(0.5);

  double speed = Constants.Intake.speed;
  double targetSpeed = 0;

  public Intake() {
    motorLeft.setInverted(Constants.Intake.leftInverted);
    motorCenter.setInverted(Constants.Intake.centerInverted);
    motorRight.setInverted(Constants.Intake.rightInverted);
    stop();
  }

  private void set(double power) {
    targetSpeed = power;
  }

  public void toggle() {
    if (targetSpeed == 0.0) {
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
    return !(targetSpeed == 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Running", isRunning());

    motorLeft.set(limiter.calculate(targetSpeed));
    motorRight.set(limiter.calculate(targetSpeed));
    motorCenter.set(limiter.calculate(targetSpeed));
  }

  public Command reverseIntakeCommand() {
    return this.run(() -> set(-0.8));
  }
}
