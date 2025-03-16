package frc.robot.Commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class IntakeAuto extends Command {
    Superstructure superstructure;
    Drive drive;
    AutoDriveCommand driver = new AutoDriveCommand(2, 0.005, 0, 1);

    public IntakeAuto(Superstructure superstructure, Drive drive) {
        this.superstructure = superstructure;
        this.drive = drive;
        addRequirements(drive, superstructure.intake, superstructure.wrist, superstructure.pivot, superstructure.elevator);
        // Add commands here
    }



    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.INTAKE);
    }

    @Override
    public void execute() {
        boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Blue);
        drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          driver.getTargetSpeeds(drive.getEstimatedPosition(), RobotState.getInstance().getIntakingPose(drive.getEstimatedPosition())),
            
           isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
    
    }

    @Override
    public boolean isFinished() {
        return superstructure.hasCoral;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
        
    }



    
}
