// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  private CANSparkMax motorWhooper =
      new CANSparkMax(Constants.Index.whooperCANID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorUpper =
      new CANSparkMax(Constants.Index.upperCANID, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput lowerBreakBeam = new DigitalInput(Constants.Index.lowerBeam);
  private DigitalInput upperBreakBeam = new DigitalInput(Constants.Index.upperBeam);
  // init variables speed and currentSpeed
  double currentSpeed = 0;
  double speed = 0.95;

  public Index() {
    motorWhooper.setInverted(Constants.Index.whooperInverted);
    motorUpper.setInverted(Constants.Index.upperInverted);
  }

  private void set(double power) {
    motorWhooper.set(power);
    motorUpper.set(power - 0.5);
    currentSpeed = power;
  }

  public void run() {
    runWhooper();
    runUpper();
  }

  public void feed() {
    motorUpper.set(0.9);
  }

  public void reverseFeed() {
    motorUpper.set(-0.9);
  }

  public void setUpper(double speed) {
    motorUpper.set(speed);
  }

  public void setWhooper(double speed) {
    motorWhooper.set(speed);
  }

  public void runUpper() {
    setUpper(Constants.Index.upperSpeed);
  }

  public void runWhooper() {
    setWhooper(Constants.Index.whooperSpeed);
  }

  public boolean isRunning() {
    return !(motorUpper.get() == 0);
  }

  public void start() {
    run();
  }

  public void stop() {
    motorWhooper.set(0);
    motorUpper.set(0);
  }

  public boolean isPrimed() {
    return getUpperSensorHit();
  }

  public boolean isInUpper() {
    return getLowerSensorHit();
  }

  public void toggle() {
    if (currentSpeed == 0.0) {
      start();
    }
  }

  public boolean getUpperSensorHit() {
    return !upperBreakBeam.get();
  }

  public boolean getLowerSensorHit() {
    return !lowerBreakBeam.get();
  }

  /*public Command manualIntake(DoubleSupplier speedSupplier) {
    return this.run(() -> set(-speedSupplier.getAsDouble()));
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Break-Beam", getUpperSensorHit());
    SmartDashboard.putBoolean("Lower Break-Beam", getLowerSensorHit());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    System.out.println(isPrimed());
    if (isPrimed() == true) {
      stop();
    }
  }
}
