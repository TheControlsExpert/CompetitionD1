package frc.robot.Commands.DriveCommands.AutoAlignAlgae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class AutoAlignAlgae extends SequentialCommandGroup {

    

    public AutoAlignAlgae(Superstructure superstructure, Drive drive, VisionSubsystem vision) {

        addCommands(new FirstPartAutoAlignAlgae(drive), new SecondPartAutoAlignAlgae(drive, vision, superstructure), new ThirdPartAutoAlignAlgae(drive, vision, superstructure));

       

    }






    
}
