package frc.robot.Commands.DriveCommands.AutoAlignSource;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class SecondPartAutoAlignSource extends Command {
    Superstructure superstructure;
    Drive drive;
    double targetPoseRotation;
    double currentPoseRotation;
    double deltaRotation;
    double speedRotation;
    double kProtation = 0.05;


    public SecondPartAutoAlignSource(Drive drive, Superstructure superstructure) {
        this.superstructure = superstructure;
        this.drive = drive;
    }


    @Override
    public void execute() {
       

        targetPoseRotation = RobotState.getInstance().getIntakingPose(drive.getEstimatedPosition()).getRotation().getRadians();
        currentPoseRotation = drive.getEstimatedPosition().getRotation().getRadians();
        //Set deltaRotation in Radians
        deltaRotation = targetPoseRotation - currentPoseRotation;
        //Calculate the smallest angle, so for example 340 becomes 20
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
        if (Math.abs(deltaRotation) <= 1) {
            speedRotation = 0;
        } else {
            speedRotation = deltaRotation * kProtation;
        }

        drive.runVelocity(new ChassisSpeeds(0.3, 0, speedRotation));




    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("is second part finishing",superstructure.hasCoral);
        return superstructure.hasCoral;
    }

   
    
}
