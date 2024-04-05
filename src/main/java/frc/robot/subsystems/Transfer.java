// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {
  /** Creates a new Transfer. */
  CANSparkMax motor = new CANSparkMax(Constants.Transfer.motorID, MotorType.kBrushless);

  private double currentSpeed;

  public Transfer() {
    motor.setInverted(Constants.Transfer.motorInverted);
    currentSpeed = 0;
    motor.setSmartCurrentLimit(20);
  }

  public void start() {
    motor.set(Constants.Transfer.motorSpeed);
    currentSpeed = Constants.Transfer.motorSpeed;
  }

  public void reverse() {
    motor.set(-Constants.Transfer.motorSpeed);
    currentSpeed = -Constants.Transfer.motorSpeed;
  }

  public void stop() {
    motor.set(0);
    currentSpeed = 0;
  }

  public void toggle() {
    if (currentSpeed == 0.0) {
      start();
    } else {
      stop();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
