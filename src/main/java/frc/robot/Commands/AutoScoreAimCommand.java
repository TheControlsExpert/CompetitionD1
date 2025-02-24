package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class AutoScoreAimCommand extends Command {
    Drive drive;
    Superstructure superstructure;
    private final AutoDriveCommand driver = new AutoDriveCommand(2, 0.03, 0, 1);
    private boolean hasGoneThroughCheckpoint = false;
    private Pose2d pastScoringPose = new Pose2d();
    
    private boolean isFlipped =
    DriverStation.getAlliance().isPresent()
    && DriverStation.getAlliance().get() == Alliance.Red;

    private CommandXboxController controller;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier rotationSupplier;

    private boolean should_auto_align = false;


    public AutoScoreAimCommand(Drive drive, Superstructure superstructure, CommandXboxController controller, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.controller = controller;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(drive);
        
    }

    @Override
    public void initialize() {
        if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level4)) {
            superstructure.setDesiredState(SuperstructureState.L4_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level3)) {
            superstructure.setDesiredState(SuperstructureState.L3_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level2)) {
            superstructure.setDesiredState(SuperstructureState.L2_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level1)) {
            superstructure.setDesiredState(SuperstructureState.L1_STOWED);
        }

        // else {
        //     superstructure.setDesiredState(SuperstructureState.L4_STOWED);
        // }

        // superstructure.setDesiredState(SuperstructureState.L4_STOWED);

    }

    @Override
    public void execute() {
    //auto-align

    if (Math.hypot(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), rotationSupplier.getAsDouble()) < 0.26 && should_auto_align) {

    
        Pose2d scoringPose = DriverStation.getAlliance().get() == Alliance.Blue ? RobotState.getInstance().getScoringPose()[0] : RobotState.getInstance().getScoringPose()[1];
        Pose2d scoringPose_offset = DriverStation.getAlliance().get() == Alliance.Blue ? RobotState.getInstance().getScoringPose_offset()[0] : RobotState.getInstance().getScoringPose_offset()[1];

        if (pastScoringPose.equals(scoringPose) && hasGoneThroughCheckpoint) {
            //bcs we are close enough, make elevator go up now
            
            


            drive.runVelocity(
               ChassisSpeeds.fromFieldRelativeSpeeds(
                    driver.getTargetSpeeds(drive.getEstimatedPosition(), scoringPose), 
                    isFlipped
                       ? drive.getRotation().plus(new Rotation2d(Math.PI))
                       : drive.getRotation()));      

        }


        else {

            hasGoneThroughCheckpoint = false;
            //make elevator go down
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
            

            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                     driver.getTargetSpeeds(drive.getEstimatedPosition(), scoringPose_offset), 
                     isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));      

            if (scoringPose_offset.minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 0.20) {
                hasGoneThroughCheckpoint = true;
            }         

        }

        pastScoringPose = scoringPose;

    }

    //override auto-aligning 
    else {

        Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());


        double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec());
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));


            ////add if we are close enough the elevator goes up    

            ////Pose2d scoringPose_offset = DriverStation.getAlliance().get() == Alliance.Blue ? RobotState.getInstance().getScoringPose_offset()[0] : RobotState.getInstance().getScoringPose_offset()[1];
            // Pose2d scoringPose = DriverStation.getAlliance().get() == Alliance.Blue ? RobotState.getInstance().getScoringPose()[0] : RobotState.getInstance().getScoringPose()[1];


            // if (scoringPose.minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 0.50) {
            //     hasGoneThroughCheckpoint = true;
            // }  

            // if (hasGoneThroughCheckpoint && pastScoringPose.equals(scoringPose)) {
            //     superstructure.setDesiredState(RobotState.getInstance().currentScoringLevel);

            // }

            // else {
            //     hasGoneThroughCheckpoint = false;
            //     superstructure.setDesiredState(SuperstructureState.HOME_UP);


            // }

            // pastScoringPose = scoringPose;

    }


        //arm and elevator logic



        //scoring logic 



     
    }

    @Override
    public boolean isFinished() {
        return controller.rightTrigger().getAsBoolean();
    }


    @Override
    public void end(boolean interrupted) {
        if (controller.rightTrigger().getAsBoolean()) {
            //put scoring state logic here
            CommandScheduler.getInstance().schedule(new ScoreCommand(superstructure));
        }

        else {
            //put homing state logic here
           superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }
        
    }


    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    
        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;
    
        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
      }
    


    
    
}
