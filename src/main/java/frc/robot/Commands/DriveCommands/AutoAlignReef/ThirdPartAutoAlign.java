package frc.robot.Commands.DriveCommands.AutoAlignReef;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

public class ThirdPartAutoAlign extends Command {
    Drive drive;
    VisionSubsystem vision;
    AutoDriveCommand2 autodriver = new AutoDriveCommand2(2, 0.03);
    double offsetX;
    double offsetY;
    boolean hasResetAtLeastOnce = false;
    Superstructure superstructure;
    CommandXboxController controller;
    //SwerveDriveOdometry odometryRelative;

    public ThirdPartAutoAlign(Drive drive, VisionSubsystem vision, Superstructure superstructure, CommandXboxController controller) { //SwerveDriveOdometry odometryRelative) {
        this.drive = drive;
        this.vision = vision;
        this.controller = controller;
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
        if (DriverStation.isAutonomous()) {
            offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET_STEPBACK * multiplierY);
            offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET_STEPBACK/2);

        }

        else {
         offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET_STEPBACK * multiplierY);
         offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET_STEPBACK);
        }
         //odometryRelative = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, drive.getRotation(), drive.lastModulePositions, new Pose);
         //hasResetAtLeastOnce = true;


        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
        }

      
    }


    @Override
    public boolean isFinished() {
        //stop aligning when we are less than 1 cm off or segev starts driving
        return  Math.hypot(offsetX, offsetY) < 0.02 || (Math.hypot(controller.getLeftX(), controller.getLeftY()) > 0.15) || (!superstructure.current_state.equals(SuperstructureState.L1_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L2_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L3_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L4_EJECTED));
    }
    

    @Override
    public void end(boolean interrupted) {
       superstructure.setDesiredState(SuperstructureState.HOME_UP);
}



    
}
