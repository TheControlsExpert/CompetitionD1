package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class ResetPivotCommand{
    Superstructure superstructure;

    

    public static Command getResetCommand(Superstructure superstructure) {
        return Commands.runOnce(() -> {superstructure.wristIO.resetPivot();}, superstructure.wrist, superstructure.pivot, superstructure.elevator, superstructure.intake);
    }

    
    
}
