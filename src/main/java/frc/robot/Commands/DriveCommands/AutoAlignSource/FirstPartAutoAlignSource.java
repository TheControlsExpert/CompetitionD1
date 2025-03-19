package frc.robot.Commands.DriveCommands.AutoAlignSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;

public  class FirstPartAutoAlignSource extends Command{
    Superstructure superstructure;
    Drive drive;
    Pose2d intakingPose = new Pose2d(16.9, 1.36, Rotation2d.fromDegrees(-50));

    AutoDriveCommand autoDriver = new AutoDriveCommand(2.1, 0.05, 0, 1);

    public FirstPartAutoAlignSource(Superstructure superstructure, Drive drive) {
     this.superstructure = superstructure;
     this.drive = drive;
    }

    @Override
    public void execute() {
        boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red); 
      
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          autoDriver.getTargetSpeeds(drive.getEstimatedPosition(), intakingPose),
            
           isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
    
                


    //SmartDashboard.putNumber("how far off are we", drive.getEstimatedPosition().minus(intakingPose)).getTranslation().getNorm();

    if (drive.getEstimatedPosition().minus(intakingPose).getTranslation().getNorm() < 3) {
        superstructure.setDesiredState(Superstructure.SuperstructureState.INTAKE);
    }
    }


    @Override
    public boolean isFinished() {
        return drive.getEstimatedPosition().minus(intakingPose).getTranslation().getNorm() < 0.025;
    }


    @Override
    public void end(boolean interrupted) {
        //superstructure.setDesiredState(Superstructure.SuperstructureState.HOME_UP);
    }
    
}
