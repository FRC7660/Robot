// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launch extends SubsystemBase {
  
  //motors
  private PWMTalonSRX launchTop = new PWMTalonSRX(11); //toplauncher
  private PWMTalonSRX launchBottom = new PWMTalonSRX(10); //bottomlauncher
  private PWMVictorSPX feeder = new PWMVictorSPX(7); //feeder

  //encoders
  private CANCoder launchTopEncoder = new CANCoder(11);
  private CANCoder launchBottomEncoder = new CANCoder(10);
  //get the speed from the encoders
  public double launchTopSpeed = launchTopEncoder.getPosition();
  public double launchBottomSpeed = launchBottomEncoder.getPosition();

  double feedSpeed = 0.2; //speed of the feeder
  double launchSpeed = 0.2; //speed of launcher
  
  

  /** Creates a new ExampleSubsystem. */
  public Launch() {}


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("launchTopSpeed", launchTopSpeed);
    SmartDashboard.putNumber("launchBottomSpeed", launchBottomSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void runFeed(){
    feeder.set(feedSpeed);
  }

  public void stopFeed(){
    feeder.set(0.0);
  }

  public void feedAndShoot() {

    //run launcher
    launchTop.set(launchSpeed);
    launchBottom.set(launchSpeed);
    
    if(launchBottomSpeed >= 1000.0) { //checks if launcher is running fast enough 
      
      feeder.set(feedSpeed);
    }
    
  }
}
