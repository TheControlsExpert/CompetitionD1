package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import org.jgrapht.alg.color.SmallestDegreeLastColoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class AutoSourcingCommand extends Command {
        Drive drive;
        Superstructure superstructure;
        frc.robot.Subsystems.Intake.Intake intake;
        private final AutoDriveCommand driver = new AutoDriveCommand(2, 0.03, 0, 1);
        private double initTime;
       
        
        private boolean isFlipped =
        DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

        boolean shouldAutoAlign = false;
    
        //private CommandXboxController controller;
        private DoubleSupplier xSupplier;
        private DoubleSupplier ySupplier;
        private DoubleSupplier rotationSupplier;
    
    
        public AutoSourcingCommand(Drive drive, Superstructure superstructure, frc.robot.Subsystems.Intake.Intake intake, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
            this.drive = drive;
            this.superstructure = superstructure;
            this.intake = intake;
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.rotationSupplier = rotationSupplier;
            addRequirements(drive, superstructure,intake);
            
        }
    
        @Override
        public void initialize() {
            //open up elevator

            superstructure.setDesiredState(SuperstructureState.HOME_DOWN);
            intake.setState(Intake.Intake_states.Bofore_First);
        }
    
        @Override
        public void execute() {
        //auto-align
    
        if (Math.hypot(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), rotationSupplier.getAsDouble()) < 0.26 && shouldAutoAlign) {
    
        
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
    }
    
    
            //arm and elevator logic
    
    
    
            //scoring logic 
    
    
    
         
        
    
    
        @Override
        public void end(boolean interrupted) {
        if (intake.CurrentintakeState.equals(Intake.Intake_states.After_First)) {
          CommandScheduler.getInstance().schedule(new IntakeCommandArm(superstructure, intake));
        }

        else {
            intake.setState(Intake.Intake_states.Empty);
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }
    }

        @Override
        public boolean isFinished() {
            SmartDashboard.putBoolean("is equal to what we dont want", intake.CurrentintakeState.equals(Intake.Intake_states.After_First));

            return intake.CurrentintakeState.equals(Intake.Intake_states.After_First);
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

    

