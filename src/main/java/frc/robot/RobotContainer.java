// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.AbsoluteDriveAdv;
import frc.robot.commands.AlignLaunchAuto;
import frc.robot.commands.LaunchWithVelo;
import frc.robot.commands.LaunchWithVeloAuton;
import frc.robot.commands.PrimeIndex;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Transfer;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem m_swerve =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerves/KrakenSwerve"));
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();
  private final Launcher m_launch = new Launcher();
  private final Climb m_climb = new Climb();
  private final Transfer m_transfer = new Transfer();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driver = new XboxController(0);
  private final XboxController coDriver = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Create commands for PathPlanner
    NamedCommands.registerCommand(
        "launch", new LaunchWithVeloAuton(m_launch, m_index, Constants.Launch.speedFarSpeaker));
    NamedCommands.registerCommand("intake", new ToggleIntake(m_intake, m_transfer));
    NamedCommands.registerCommand("index", new PrimeIndex(m_index, m_transfer));
    NamedCommands.registerCommand(
        "align-launch", new AlignLaunchAuto(m_swerve, m_launch, m_index, 5300, 1));
    NamedCommands.registerCommand("reverse intake", m_intake.reverseIntakeCommand());

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    // Configure the trigger bindings
    configureBindings();

    int povAngle = driver.getPOV();

    Trigger driverUp = new Trigger(() -> (povAngle >= 315 || povAngle <= 45));
    Trigger driverDown = new Trigger(() -> (povAngle >= 135 && povAngle <= 225));
    Trigger driverLeft = new Trigger(() -> (povAngle >= 225 && povAngle <= 315));
    Trigger driverRight = new Trigger(() -> (povAngle >= 45 && povAngle <= 135));

    AbsoluteDrive closedAbsoluteDrive =
        new AbsoluteDrive(
            m_swerve,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(-driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> -driver.getRightX(),
            () -> -driver.getRightY());

    AbsoluteDriveAdv advancedDrive =
        new AbsoluteDriveAdv(
            m_swerve,
            () -> MathUtil.applyDeadband(-driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(-driver.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> driverUp.getAsBoolean(),
            () -> driverDown.getAsBoolean(),
            () -> driverLeft.getAsBoolean(),
            () -> driverRight.getAsBoolean());

    m_swerve.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : advancedDrive);

    // -m_index.setDefaultCommand(m_index.manualIntake(coDriver::getRightY));

    m_climb.setDefaultCommand(m_climb.setWinchCommand(coDriver::getLeftY));

    // add auto options
    m_chooser.setDefaultOption("Test Drive", m_swerve.getAutonomousCommand("Test Drive"));

    m_chooser.addOption("Mid 2 Piece", m_swerve.getAutonomousCommand("Mid 2 Piece"));
    m_chooser.addOption("Turn Auto", m_swerve.getAutonomousCommand("Turn Auto"));
    m_chooser.addOption("Drive and Turn", m_swerve.getAutonomousCommand("drive and turn"));
    m_chooser.addOption("1 Centerline", m_swerve.getAutonomousCommand("1 Centerline"));
    m_chooser.addOption("5 Centerline", m_swerve.getAutonomousCommand("5 Centerline"));
    m_chooser.addOption("Close 2", m_swerve.getAutonomousCommand("Close 2"));
    m_chooser.addOption("Just Chill", m_swerve.noAuto());
    m_chooser.addOption(
        "Simple Shoot Center", m_swerve.getAutonomousCommand("Simple Shoot Center"));
    m_chooser.addOption(
        "Simple Shoot Safe Blue", m_swerve.getAutonomousCommand("Simple Shoot Safe Blue"));
    m_chooser.addOption("Simple Shoot Amp", m_swerve.getAutonomousCommand("Simple Shoot Amp"));
    m_chooser.addOption(
        "Simple Shoot Safe Red", m_swerve.getAutonomousCommand("Simple Shoot Safe Red"));

    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    leftBumper.onTrue(new ToggleIntake(m_intake, m_transfer));

    JoystickButton rb = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    rb.whileTrue(new LaunchWithVelo(m_launch, m_index, -1500, false));

    JoystickButton coRB = new JoystickButton(coDriver, XboxController.Button.kRightBumper.value);
    coRB.onTrue(new PrimeIndex(m_index, m_transfer));

    Trigger lt = new Trigger(() -> driver.getLeftTriggerAxis() >= 0.05);
    lt.whileTrue(m_swerve.alignCommand());

    Trigger rt = new Trigger(() -> driver.getRightTriggerAxis() >= 0.05);
    rt.whileTrue(new LaunchWithVelo(m_launch, m_index, 5200, false));

    JoystickButton x = new JoystickButton(driver, XboxController.Button.kX.value);
    x.whileTrue(m_swerve.updatePositionCommand());

    JoystickButton a = new JoystickButton(driver, XboxController.Button.kA.value);
    a.whileTrue(new LaunchWithVelo(m_launch, m_index, 2050, false));

    JoystickButton y = new JoystickButton(driver, XboxController.Button.kY.value);
    y.whileTrue(m_intake.reverseIntakeCommand());

    JoystickButton b = new JoystickButton(driver, XboxController.Button.kB.value);
    b.whileTrue(m_swerve.setRotationCommand(180));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // autonselect
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    m_swerve.setMotorBrake(brake);
  }

  public void updatePose() {
    m_swerve.updatePositionCommand();
  }

  public void dropLauncher() {
    m_launch.releaseLauncher();
  }

  public void lockLauncher() {
    m_launch.lockLauncher();
  }
}
