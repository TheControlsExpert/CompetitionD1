package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class AutonomousL4 extends Command {
    private final AutoDriveCommand driver = new AutoDriveCommand(2.5, 0.03, 0, 1);
    Drive drive;
    Pose2d trackingPose;
    double initTime;
    Superstructure superstructure; 


    public AutonomousL4(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }


    @Override
    public void execute() {

        boolean isFlipped =
        DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;


         trackingPose = isFlipped ? RobotState.PositionGetter.get(ScoringPosition.A)[1] : RobotState.PositionGetter.get(ScoringPosition.A)[0];

        
 
            
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          driver.getTargetSpeeds(drive.getEstimatedPosition(), trackingPose),
           isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }


    @Override
    public boolean isFinished() {
        return Math.abs(drive.getEstimatedPosition().getTranslation().minus(trackingPose.getTranslation()).getNorm()) < 0.075 || Timer.getFPGATimestamp() - initTime > 10;
    }


    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
        superstructure.setDesiredState(SuperstructureState.L4_EJECTED);    
    }
}