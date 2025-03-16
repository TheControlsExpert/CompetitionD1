package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class OH_SHIT extends Command {
    Superstructure superstructure;

    public OH_SHIT(Superstructure superstructure) {
        this.superstructure = superstructure;
    }


    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {
        superstructure.hasCoral = false;
        superstructure.hasAlgae = false;
        superstructure.isEjectingManually = false;
        superstructure.current_state = SuperstructureState.HOME_UP;
        superstructure.desired_state = SuperstructureState.HOME_UP;
    }


   
    
}
