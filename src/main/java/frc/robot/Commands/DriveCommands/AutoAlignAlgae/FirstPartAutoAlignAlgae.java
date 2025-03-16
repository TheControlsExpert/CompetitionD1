package frc.robot.Commands.DriveCommands.AutoAlignAlgae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand;
import frc.robot.Subsystems.Drive.Drive;

public class FirstPartAutoAlignAlgae extends Command {
    Drive drive;
    AutoDriveCommand autoDriver = new AutoDriveCommand(2, 0.05, 0, 1);



    public FirstPartAutoAlignAlgae(Drive drive) {
        this.drive = drive;
        addRequirements(drive);

    }


    @Override
    public void execute() {
         boolean isFlipped = DriverStation.getAlliance().get().equals(Alliance.Red);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                autoDriver.getTargetSpeeds(drive.getEstimatedPosition(), RobotState.getInstance().getClosestAlgaeReef(drive.getEstimatedPosition())),
                  
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
    }


    @Override
    public boolean isFinished() {
        double  deltarotation = Math.abs(Units.radiansToDegrees(MathUtil.angleModulus(RobotState.getInstance().getScoringPose().getRotation().minus((drive.getEstimatedPosition()).getRotation()).getRadians())));
        SmartDashboard.putNumber("is finished aligning", deltarotation );
        return deltarotation < 0.5 && drive.getEstimatedPosition().minus(RobotState.getInstance().getClosestAlgaeReef(drive.getEstimatedPosition())).getTranslation().getNorm() < 0.025;
    }


    
}
