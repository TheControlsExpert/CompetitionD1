package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class ProcessorPrepCommand extends Command {
    Superstructure superstructure;
    CommandXboxController controller;


    public ProcessorPrepCommand(Superstructure superstructure, CommandXboxController controller) {
        this.superstructure = superstructure;
        this.controller = controller;
    }
  

    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.PROCESSOR);
    }


    @Override
    public void end(boolean interrupted) {
        if (!controller.rightTrigger().getAsBoolean()) {
            superstructure.setDesiredState(SuperstructureState.HOME_ALGAE);

        }
    }


    
    
}
