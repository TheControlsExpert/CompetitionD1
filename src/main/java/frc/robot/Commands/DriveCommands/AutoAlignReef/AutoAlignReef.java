package frc.robot.Commands.DriveCommands.AutoAlignReef;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class AutoAlignReef extends SequentialCommandGroup {

    public AutoAlignReef(Drive drive, VisionSubsystem vision, Superstructure superstructure, ScoringPosition scoringPosition, CommandXboxController controller) {
        
        addCommands(new FirstPartAutoAlign(drive, superstructure, scoringPosition), new SecondPartAutoAlign(drive, vision, superstructure), new EjectCommand(superstructure, drive, vision), new ThirdPartAutoAlign(drive, vision, superstructure, controller));    }


    public AutoAlignReef(Drive drive, VisionSubsystem vision, Superstructure superstructure, CommandXboxController controller) {
        addCommands(new FirstPartAutoAlign(drive, superstructure), new SecondPartAutoAlign(drive, vision, superstructure), new EjectCommand(superstructure, drive, vision), new ThirdPartAutoAlign(drive, vision, superstructure, controller));
    }
    
}
