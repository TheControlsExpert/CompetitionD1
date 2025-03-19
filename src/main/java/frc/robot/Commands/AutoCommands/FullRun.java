// package frc.robot.Commands.AutoCommands;

// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotContainer;
// import frc.robot.Commands.DriveCommands.AutoAlignReef.AutoAlignReef;
// import frc.robot.Subsystems.Drive.Drive;
// import frc.robot.Subsystems.Superstructure.Superstructure;
// import frc.robot.Subsystems.Vision.VisionSubsystem;

// public class FullRun extends SequentialCommandGroup {
    

//     public FullRun(Drive drive, Superstructure superstructure, VisionSubsystem  vision) {
       
//         addCommands(drive.getPathFollowingCommand("3 Piece Bottom PT1"),
//                     new AutoAlignReef(drive, vision, superstructure, RobotContainer.ScoringPosition.K));
                   
//                     //drive.getPathFollowingCommand("3 Piece Bottom PT2"),
//                     //new IntakeAuto(superstructure, drive),
//                     //drive.getPathFollowingCommand("3 Piece Bottom PT3"),
//                     //new AutoAlignReef(drive, vision, RobotContainer.ScoringPosition.I),
//                     //new IntakeAuto(superstructure, drive));

//     }
    
// }
