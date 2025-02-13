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
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drive.Drive;

public class AutoSourcingCommand extends Command {
        Drive drive;
        private final AutoDriveCommand driver = new AutoDriveCommand(2, 0.03, 0, 1);
        private boolean hasGoneThroughCheckpoint = false;
        private Pose2d pastScoringPose = new Pose2d();
        
        private boolean isFlipped =
        DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    
        //private CommandXboxController controller;
        private DoubleSupplier xSupplier;
        private DoubleSupplier ySupplier;
        private DoubleSupplier rotationSupplier;
    
    
        public AutoSourcingCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
            this.drive = drive;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.rotationSupplier = rotationSupplier;
            addRequirements(drive);
            
        }
    
        @Override
        public void initialize() {
            //open up elevator
            
    
        }
    
        @Override
        public void execute() {
        //auto-align
    
        if (Math.hypot(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), rotationSupplier.getAsDouble()) < 0.26) {
    
        
            Pose2d intakingPose = RobotState.getInstance().getIntakingPose(drive.estimatedPose);

            
    
           
    
                drive.runVelocity(
                   ChassisSpeeds.fromFieldRelativeSpeeds(
                        driver.getTargetSpeeds(drive.getEstimatedPosition(), intakingPose), 
                        isFlipped
                           ? drive.getRotation().plus(new Rotation2d(Math.PI))
                           : drive.getRotation()));      
    
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
    
            
    
           
    
        }

        //if we are close enough to source move subsystems to their right place
        
    }
    
    
            //arm and elevator logic
    
    
    
            //scoring logic 
    
    
    
         
        
    
    
        @Override
        public void end(boolean interrupted) {
          //intaking logic here
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

    

