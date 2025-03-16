package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.DirectionREEF;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class SecondPartAutoAlign extends Command {
    Drive drive;
    VisionSubsystem vision;
    AutoDriveCommand2 autodriver = new AutoDriveCommand2(2, 0.03);

    public SecondPartAutoAlign(Drive drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
        
    }

    @Override
    public void execute() {

        double multiplierY = RobotState.getInstance().getScoringDirection().equals(DirectionREEF.RIGHT) ? -1 : 1;
        double offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET * multiplierY);
        double offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET);


        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
    
     
    }
    
}
