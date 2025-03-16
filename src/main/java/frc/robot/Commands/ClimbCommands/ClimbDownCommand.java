package frc.robot.Commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class ClimbDownCommand extends Command {
    Climb climb;

    public ClimbDownCommand(Climb climb) {
        this.climb = climb;
    }


    @Override
    public void execute() {
        climb.unClimb();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.Climb_Stop();
    }
    
}
