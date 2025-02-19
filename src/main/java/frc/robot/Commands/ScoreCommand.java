package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class ScoreCommand extends Command {
    private final Superstructure superstructure;
    public ScoreCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }


    @Override
    public void initialize() {
        superstructure.setDesiredState(RobotState.getInstance().currentScoringLevel);
    }

    @Override
    public boolean isFinished() {
        return !superstructure.hasCoral();
    }


    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }

    
    
}
