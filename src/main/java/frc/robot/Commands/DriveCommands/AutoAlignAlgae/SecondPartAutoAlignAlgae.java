package frc.robot.Commands.DriveCommands.AutoAlignAlgae;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand2;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.RobotState.AlgaeLevel;
import frc.robot.RobotState.DirectionREEF;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class SecondPartAutoAlignAlgae extends Command {
    Drive drive;
    VisionSubsystem vision;
    AutoDriveCommand2 autodriver = new AutoDriveCommand2(2, 0.03);
    double offsetX;
    double offsetY;
    boolean hasResetAtLeastOnce = false;
    Superstructure superstructure;
    //SwerveDriveOdometry odometryRelative;

    public SecondPartAutoAlignAlgae(Drive drive, VisionSubsystem vision, Superstructure superstructure) { //SwerveDriveOdometry odometryRelative) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        //this.odometryRelative = odometryRelative;
        addRequirements(drive, superstructure.intake, superstructure.wrist, superstructure.elevator, superstructure.pivot);
        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if (vision.reeftransformX != 0 && vision.reeftransformY != 0) {

        

        double multiplierY = RobotState.getInstance().getScoringDirection().equals(DirectionREEF.RIGHT) ? -1 : 1;
         offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET_ALGAE * multiplierY);
         offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET_ALGAE);

         //odometryRelative = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, drive.getRotation(), drive.lastModulePositions, new Pose);
         hasResetAtLeastOnce = true;


        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
        }

        if (RobotState.getInstance().getAlgaeLevel(drive.getEstimatedPosition()).equals(AlgaeLevel.L2)){

        superstructure.setDesiredState(SuperstructureState.L2_ALGAE);
        }

        else {
        superstructure.setDesiredState(SuperstructureState.L3_ALGAE);    

        }

    }


    @Override
    public boolean isFinished() {
        //in autonomous, stop aligning when we are less than 1 cm off
        return  Math.hypot(offsetX, offsetY) < 0.01 && superstructure.hasAlgae;
    }
    

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }
}



    
}
