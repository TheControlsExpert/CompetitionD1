package frc.robot.Commands.DriveCommands.AutoAlignReef;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand;
import frc.robot.Robot.ReefMode;
import frc.robot.Robot.levelscore;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class FirstPartAutoAlign extends Command {
    Drive drive;
    Superstructure superstructure;
    ScoringPosition position;
    boolean shouldSet = false;
    AutoDriveCommand autoDriver = new AutoDriveCommand(2.1, 0.05, 0, 1);


    public FirstPartAutoAlign(Drive drive, Superstructure superstructure) {
        shouldSet = false;
        this.drive = drive;
        this.superstructure = superstructure;
        addRequirements(drive, superstructure.intake, superstructure.elevator, superstructure.wrist, superstructure.pivot);
        
    }

    public FirstPartAutoAlign(Drive drive, Superstructure superstructure, ScoringPosition position) {
        this.position = position;
        shouldSet = true;
        this.drive = drive;
        this.superstructure = superstructure;
        addRequirements(drive, superstructure.intake, superstructure.elevator, superstructure.wrist, superstructure.pivot);
        
    }

    @Override
    public void initialize() {
        if (shouldSet) {
        Robot.CurrnetScoringPosition = position;
        }
    }


    @Override
    public void execute() {

        
        boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                autoDriver.getTargetSpeeds(drive.getEstimatedPosition(), RobotState.getInstance().getScoringPose()),
                  
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));

        if (drive.getEstimatedPosition().minus(RobotState.getInstance().getScoringPose()).getTranslation().getNorm() < 2) {
            if (Robot.CurrnetLevelPosition.equals(levelscore.Level4)) {
                superstructure.setDesiredState(SuperstructureState.L4_STOWED);
            }
    
            else if (Robot.CurrnetLevelPosition.equals(levelscore.Level3)) {
                superstructure.setDesiredState(SuperstructureState.L3_STOWED);
            }
    
            else if (Robot.CurrnetLevelPosition.equals(levelscore.Level2)) {
                superstructure.setDesiredState(SuperstructureState.L2_STOWED);
            }
    
            else if (Robot.CurrnetLevelPosition.equals(levelscore.Level1)) {
                superstructure.setDesiredState(SuperstructureState.L1_STOWED);
            }
        }              

        
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }
    }

    @Override
    public boolean isFinished() {
        double  deltarotation = Math.abs(Units.radiansToDegrees(MathUtil.angleModulus(RobotState.getInstance().getScoringPose().getRotation().minus((drive.getEstimatedPosition()).getRotation()).getRadians())));
        SmartDashboard.putNumber("is finished aligning", deltarotation );
        return deltarotation < 1 && drive.getEstimatedPosition().minus(RobotState.getInstance().getScoringPose()).getTranslation().getNorm() < 0.05;
    }

    
}
