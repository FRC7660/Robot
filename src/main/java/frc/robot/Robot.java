/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



//Imports that's all

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.*;

//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import frc.robot.subsystems.Limelight;


public class Robot extends TimedRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  

  //controllers
  private final XboxController joystick1 = new XboxController(0);
  // private final XboxController joystick2 = new XboxController(1);

  // motors controllers
  public static TalonSRX left1 = new TalonSRX(2);
  private final WPI_VictorSPX left2 = new WPI_VictorSPX(3);
  private final TalonSRX right1 = new TalonSRX(4);
  private final WPI_VictorSPX right2 = new WPI_VictorSPX(5);

  // private CANSparkMax winchMotor = new CANSparkMax(12, MotorType.kBrushless);

  // drive
  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(left2);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(right2);
  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  //changed to always true
  private boolean yeetMode = false;

  private PIDController pidController;

  private Limelight llClass;

  // transport
  private final Transport transport = new Transport();

  // hangerSolenoid prior numbers (0, 1)
  // private final DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 6);
  //private DoubleSolenoid hangerSolenoid = new DoubleSolenoid(5, 7);
  //time_buddhist_cronology
  // limelight


  public double limelightHeight = 2.58; // feet
  public double targetHeight = 7; // also feet
  public double limelightAngle = 0;

  // launcher helper variables, distance equals distance from the sun
  private double distance = 0;
  public double deltaHeight = 0;

  // These two components are the motor and PID controller for the shooter
  PWMTalonSRX pwmTalonSRX;
  PIDSubsystem pidSubsystem;

  // The value of these three constants will need to be determined later
  double Kp = 0.04f;
  double Kd = 0.0008f;
  double Ki = 0.0004f;
  double min_command = 0.03f;
  // Milford speeds
  /*
   * double Kp = 0.04f; double Kd = 0.0008f; double Ki = 0.0004f; double
   * min_command = 0.03f;
   */

  int neutralZone = 0;

  private boolean driverInputAccepted = true;

  double autonTimer = 400;

  double lastLaunchPress = 0;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    llClass = new Limelight();

    //CameraServer.startAutomaticCapture();

    transport.Reset();

    pidController = new PIDController(Kp, Ki, Kd);
    pidController.setTolerance(0.3f);

    left1.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);

    transport.InitTransport();
    
  }

  @Override
  public void robotPeriodic() {
    if (super.isAutonomous()) {
      return;
    }
    llClass.periodic();
    transport.UpdateTransport();
    Drive();
    ArmControl();
    CheckArm();
    // CalculateDistance();
    align();

    lastLaunchPress = joystick1.getRightTriggerAxis();
  }

  private void CheckArm() {
    
  }

  private void Drive() {
    SmartDashboard.putBoolean("Yeet", yeetMode);

    if (driverInputAccepted) {
      if (joystick1.getYButtonPressed()) {
        yeetMode = !yeetMode;
      }
      if (yeetMode) {
        if(joystick1.getAButton()){
          drive.arcadeDrive(joystick1.getLeftX() * 1.0f, joystick1.getLeftY() * 1.0f, false);
        }
        else{
            drive.arcadeDrive(joystick1.getLeftX() * 1.0f, -joystick1.getLeftY() * 1.0f, false);
        }
      } 
      else {
        if(joystick1.getAButton()){
          drive.arcadeDrive(joystick1.getLeftX() * 0.75f, joystick1.getLeftY() * 0.9f, false);
        }
        else{
          drive.arcadeDrive(joystick1.getLeftX() * 0.75f, -joystick1.getLeftY() * 0.90f, false);
        }
      }
      left1.set(ControlMode.PercentOutput, left2.getMotorOutputPercent());
      right1.set(ControlMode.PercentOutput, right2.getMotorOutputPercent());

    }
  }

  private void Rotate(double turn) {
    final double min = 0.4f;

    if (turn > 1) {
      turn = 1;
    }
    if (turn < -1) {
      turn = -1;
    }
    if (turn < min && turn > 0) {
      turn = min;
    }
    if (turn > -min && turn < 0) {
      turn = -min;
    }
    drive.arcadeDrive(0, turn, true);
    left1.set(ControlMode.PercentOutput, left2.getMotorOutputPercent());
    right1.set(ControlMode.PercentOutput, right2.getMotorOutputPercent());
  }

  private void align(){
    if(joystick1.getLeftTriggerAxis() > 0.1){
      double tx = llClass.tx();
      if(tx < -4){
        drive.arcadeDrive(-0.6, 0);
      }
      else if(tx > -4 && tx < -1){
        drive.arcadeDrive(-0.45, 0);
      }
      else if(tx < 4 && tx > 1.2){
        drive.arcadeDrive(0.4, 0);
      }
      else if(tx > 4){
        drive.arcadeDrive(0.6, 0);
      }

    }
  }

//This is the codeing system for our actuall programing for our auton.

  // private void CheckArm()
{

  
    // Create timer so arm does not activate until endgame
    //if(joystick2.getRightTriggerAxis() > 0.5f)  }
      //hangerSolenoid.set(Value.kForward); } else
    //if(joystick2.getRightTriggerAxis() < 0.5f) {
      //hangerSolenoid.set(Value.kReverse); }
    //if(joystick2.getLeftTriggerAxis() > 0.05f) } // winchMotor.set(-0.50f); }
    }  

  public void PIDSearch() {

  }
  

  private void ArmControl()
  {

  }

  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();

    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }
  double slalom [] = {5,-1,5,-2,15,-2,5,-1,5,-1,5,-1,5,-1,5,-2,15,-2,5,-1,5};
  double slalomTimes [] = {56,-1,62,-2,179,-2,62,-1,62,-1,62,0,0,0,0,0,0,0,-1,62,-1,62,-2,179,-2,62,-1,62,0};
  //double slalomTimes [] = {2,-1,15,-2,192,-2,15,-1,22,-1,15,-1,22,-1,15,-2,192,-2,15,-1,22,0};
  int index = 0;
  int leftTurnTicks = 38;
  int rightTurnTicks = 38;
  //90 deg.: 45 ticks 0.799 speed
  //2 ft: 21 ticks 1 speed
  //3 ft: 30 ticks 1 speed
  //4 ft: 41 ticks 1 speed
  //5 ft: 52 ticks 1 speed
  //20 ft: 217 ticks
  //THEORY: Pattern-after 3 ft, 10 times the number of feet + number of feet - 3;
  //Battery 4 -- 15 ft: 170 ticks 1 speed __ 5 ft:57 ticks 1 speed


//PS: Never use the following code. Just don't. It's useless. Just like the person seeing this.
//PS: NEVER UNDO THE COMMENTING!.!.!.
  @Override
  public void autonomousPeriodic() 
  { 
    // if(autonTimer < 400 - slalomTimes[index])
    // { 
      
    // }
  

    //The statements below are barely place holders, 
//we do not know how rotate operates, questions are
//1. Which direction (positive or negitive)
//2. Does it complete the turn ordo we have to continue 
//turning.Beans
    // if(slalomTimes[index] == -2)
    // {
    //     if (rightTurnTicks >= 1)
    //     {
    //         drive.arcadeDrive(0,0.799);
    //         rightTurnTicks -= 1;
    //     }
    //     else if (rightTurnTicks == 0)
    //     {
    //         drive.stopMotor();
    //         drive.arcadeDrive(0,0);
    //         index++;
    //         rightTurnTicks += 38;
    //     }
    // }
    // else if(slalomTimes[index] == -1)
    // {
    //     if(leftTurnTicks >= 1)
    //     {
    //         drive.arcadeDrive(0,-0.799);
    //         leftTurnTicks -= 1;
    //     }
    //     else if(leftTurnTicks == 0)
    //     {
    //         drive.stopMotor();
    //         drive.arcadeDrive(0,0);
    //         index++;
    //         leftTurnTicks += 38;
    //     }
    // }
    // else if(slalomTimes[index] >= 1)
    // {
    //     drive.arcadeDrive(1,0.3);
    //     slalomTimes[index]--;
    // }
    // else if(slalomTimes[index] == 0)
    // {
    //     drive.stopMotor();
    //     //drive.arcadeDrive(0,0);

    //     if( index < slalomTimes.length)
    //     {
    //       index++;
    //     }
    // } 
    autonTimer--;
    if (autonTimer <= 0 && autonTimer > -200)
    {
      // armSolenoid.set(Value.kForward);
      //transport.StopLaunch();
      drive.arcadeDrive(-0.45f, 0);
      left1.set(ControlMode.PercentOutput, left2.getMotorOutputPercent());
      right1.set(ControlMode.PercentOutput, right2.getMotorOutputPercent());
    }
    else if (autonTimer <= -200)
    {
      transport.StopLaunch();
      drive.stopMotor();
      left1.set(ControlMode.PercentOutput, left2.getMotorOutputPercent());
      right1.set(ControlMode.PercentOutput, right2.getMotorOutputPercent());
    } 
    else
    {
      transport.StartLaunch(0.75f);
    }

    switch (m_autoSelected) 
    {
      
      case kCustomAuto:
        // Put custom auto code here  
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void disabledInit()
  {
    autonTimer = 400f;
  }

  @Override
  public void teleopPeriodic()
  {

  }

  @Override
  public void testPeriodic()
  {
  
  }
}
