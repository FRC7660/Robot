// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {

  public Drive() {
    leftGroup.setInverted(true);
  }

  private CANSparkMax leftFront;
  private CANSparkMax rightFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightBack;

  public void setMotors() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(3, MotorType.kBrushless);
    rightBack = new CANSparkMax(4, MotorType.kBrushless);
  }

  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftBack, leftFront);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightFront, rightBack);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  public void setTankDrive(DoubleSupplier lSpeed, DoubleSupplier rSpeed, Double pOutput) {

    driveTrain.tankDrive(lSpeed.getAsDouble() * pOutput, rSpeed.getAsDouble() * pOutput);
  }

  public void setArcadeDrive(Double speed, Double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
