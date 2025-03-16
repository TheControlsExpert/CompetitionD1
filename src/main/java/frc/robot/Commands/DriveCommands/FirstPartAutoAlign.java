package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Robot.ReefMode;
import frc.robot.Subsystems.Drive.Drive;

public class FirstPartAutoAlign extends Command {
    Drive drive;
    AutoDriveCommand autoDriver = new AutoDriveCommand(2, 0.03, 0, 1);


    public FirstPartAutoAlign(Drive drive) {
        
        this.drive = drive;
        addRequirements(drive);
        
    }


    @Override
    public void execute() {
        boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                autoDriver.getTargetSpeeds(drive.getEstimatedPosition(), RobotState.getInstance().getScoringPose()),
                  
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));

        
    }

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getScoringPose().minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 0.5 && RobotState.getInstance().getScoringPose().getRotation().minus((drive.getEstimatedPosition()).getRotation()).getDegrees() < 5;
    }
    
}
