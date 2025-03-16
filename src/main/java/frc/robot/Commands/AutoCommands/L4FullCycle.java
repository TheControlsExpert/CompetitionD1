package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class L4FullCycle extends SequentialCommandGroup {

    public L4FullCycle(Superstructure superstructure) {
        addCommands(Commands.runOnce(() -> {superstructure.setDesiredState(SuperstructureState.L4_STOWED);}, superstructure.intake, superstructure.pivot, superstructure.elevator, superstructure.wrist), 
                    new EjectCommand(superstructure));    
    }
    
}
