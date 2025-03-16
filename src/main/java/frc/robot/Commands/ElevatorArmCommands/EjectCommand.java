package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class EjectCommand extends Command {
    Superstructure superstructure;
    double initTime;

    public EjectCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure.intake, superstructure.wrist, superstructure.pivot, superstructure.elevator);
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        if (superstructure.current_state.equals(SuperstructureState.L1_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L1_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L2_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L2_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L3_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L3_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L4_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.PROCESSOR)) {
            superstructure.setDesiredState(SuperstructureState.ALGAE_EJECT);
        }

        else if (!superstructure.desired_state.equals(SuperstructureState.L1_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L2_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L3_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L4_STOWED) && !superstructure.desired_state.equals(SuperstructureState.PROCESSOR) ) {
            superstructure.isEjectingManually = true;
        }

        


    }

    @Override
    public void execute() {
        if (!superstructure.isEjectingManually) {
        if (superstructure.current_state.equals(SuperstructureState.L1_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L1_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L2_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L2_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L3_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L3_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L4_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.PROCESSOR)) {
            superstructure.setDesiredState(SuperstructureState.ALGAE_EJECT);
        }

       
    }

     
    }


    @Override
    public boolean isFinished() {
        if (superstructure.current_state.equals(SuperstructureState.L1_EJECTED) || superstructure.current_state.equals(SuperstructureState.L2_EJECTED) || superstructure.current_state.equals(SuperstructureState.L3_EJECTED) || superstructure.current_state.equals(SuperstructureState.L4_EJECTED)) {
             return true;
        }

        return false; 
       
    }
     
    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
        superstructure.isEjectingManually = false;
        superstructure.hasAlgae = false;
        superstructure.hasCoral = false;
    }
    
}
