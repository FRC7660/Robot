// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  private CANSparkMax climbWench = new CANSparkMax(8, MotorType.kBrushless);
  private DoubleSolenoid climbSolenoid = new DoubleSolenoid(12, PneumaticsModuleType.CTREPCM, 5, 4);
  private boolean isExtended = false;

  public void extendSolenoid(){
    climbSolenoid.set(Value.kForward);
  }

  public void setIsExtended(boolean input){
    isExtended = input;
  }

  public boolean getIsExtended(){
    return isExtended;
  }

  public void retractSolenoid(){
    climbSolenoid.set(Value.kReverse);
  }

  public void solenoidOff(){
    climbSolenoid.set(Value.kOff);
  }

  public void runWench(DoubleSupplier speed){
    climbWench.set(speed.getAsDouble() * 0.4f);
  }

  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
