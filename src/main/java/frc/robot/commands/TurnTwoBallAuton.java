package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSolenoid;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Limelight;

public class TurnTwoBallAuton extends SequentialCommandGroup {
    
    public TurnTwoBallAuton(Launch launch, IntakeSolenoid intakeSolenoid, Index index, Drive drive, Limelight limelight, Intake intake){
        
        addCommands(
            new SwitchIntakeSolenoid(intakeSolenoid),
            new RunIntakeAuton(intake),
            new DriveDistance(drive, 48.0),
            new TurnDegrees(drive, -10.0),
            new SwitchAngle(launch),
            new ShootManual(launch, index, 1950, 9),
            new DriveDistance(drive, 12)
            // new TurnDegrees(drive, -180.0), //test turning the other direction, should be in start pos
            // new DriveDistance(drive, 36)//drive forward about 3 feet
            //new DriveDistance(drive, 36)//reverse, should be at start pos
        );
    }
}
