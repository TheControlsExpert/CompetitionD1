package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class AutoAlignReef extends SequentialCommandGroup {

    public AutoAlignReef(Drive drive, VisionSubsystem vision) {
        addCommands(new FirstPartAutoAlign(drive), new SecondPartAutoAlign(drive, vision));
    }
    
}
