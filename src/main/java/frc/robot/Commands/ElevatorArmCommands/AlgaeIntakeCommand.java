package frc.robot.Commands.ElevatorArmCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.RobotState.AlgaeLevel;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class AlgaeIntakeCommand extends Command {
    Superstructure superstructure;
    Drive drive;
    CommandXboxController controller;

    public AlgaeIntakeCommand(Superstructure superstructure, Drive drive, CommandXboxController controller) {
        this.superstructure = superstructure;
        this.drive = drive;
        this.controller = controller;
        addRequirements(superstructure.intake, superstructure.elevator, superstructure.wrist, superstructure.pivot);
    }


    @Override
    public void execute() {
        if (RobotState.getInstance().getAlgaeLevel(drive.getEstimatedPosition()).equals(AlgaeLevel.L3)) {
            superstructure.setDesiredState(SuperstructureState.L3_ALGAE);
        }
        
        else {
            superstructure.setDesiredState(SuperstructureState.L2_ALGAE);
        }
    }

  

    @Override
    public void end(boolean interrupted) {
        if (!controller.rightTrigger().getAsBoolean()) {

        

        if (superstructure.hasAlgae) {
       superstructure.setDesiredState(SuperstructureState.HOME_ALGAE);
        }

        else {
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }
    }
    }
}


    

