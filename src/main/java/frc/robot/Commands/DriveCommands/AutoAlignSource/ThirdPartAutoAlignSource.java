package frc.robot.Commands.DriveCommands.AutoAlignSource;

import org.jgrapht.alg.shortestpath.CHManyToManyShortestPaths;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class ThirdPartAutoAlignSource extends Command {

    Drive drive;
    Superstructure superstructure;
    double initTime;


    public ThirdPartAutoAlignSource(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        drive.runVelocity(new ChassisSpeeds(-4, 0, 0));
    }


    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - initTime > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }
    
}
