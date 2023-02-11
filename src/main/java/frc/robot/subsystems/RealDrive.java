// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import java.util.function.DoubleSupplier;

public class RealDrive extends Drive {

  public RealDrive() {
    leftGroup.setInverted(true);
  }

  private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);

  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftBack, leftFront);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightFront, rightBack);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  @Override
  public void setTankDrive(DoubleSupplier lSpeed, DoubleSupplier rSpeed, double pOutput) {

    driveTrain.tankDrive(lSpeed.getAsDouble() * pOutput, rSpeed.getAsDouble() * pOutput);
  }

  public void setArcadeDrive(double speed, double rotation) {

    driveTrain.arcadeDrive(speed, rotation);
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
