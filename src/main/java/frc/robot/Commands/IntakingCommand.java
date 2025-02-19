package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.Intake_states;

public class IntakingCommand extends Command {
    private final Intake intake;

    public IntakingCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
        intake.setState(Intake_states.Bofore_First);
    }



    @Override
    public boolean isFinished() {
        return intake.CurrentintakeState.equals(Intake_states.Ready);
    }



    @Override
    public void end(boolean interrupted) {
        if (intake.CurrentintakeState.equals(Intake_states.Bofore_First)) {
            intake.setState(Intake_states.Empty);
            
        }
    }
    
}
