package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class ResetWristCommand extends Command{
    Superstructure superstructure;
    boolean startedHoming = false;
    double initTime;

    public ResetWristCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure.intake, superstructure.pivot, superstructure.elevator, superstructure.wrist);
    }


    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.FIXING_WRIST);
        startedHoming = false;
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!startedHoming) {
            superstructure.wristIO.setWristPosition(-15.5);
            if (Math.abs(superstructure.wristInputs.wristAngle + 15.5) < WristConstants.toleranceWrist) {
                startedHoming = true;
            }
        }


        else {
           
            superstructure.wristIO.setOutputOpenLoopWrist(0.05);
           
           
        }


    }


    @Override
    public boolean isFinished() {
        return superstructure.Reset_ready || Timer.getFPGATimestamp() - initTime > 6;
        
    }


    @Override
    public void end(boolean interrupted) {
        superstructure.wristIO.setOutputOpenLoopWrist(0);
        if (!interrupted && superstructure.Reset_ready) {
        superstructure.wristIO.resetWrist();
        }
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }
    
}
