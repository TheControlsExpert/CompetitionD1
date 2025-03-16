package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Robot.ReefMode;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class IntakeCommand extends Command {
    Superstructure superstructure;
    boolean startcounting = false;
    double initTime = 0;
    
    public IntakeCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure.intake, superstructure.elevator, superstructure.pivot, superstructure.wrist);

    }

    
    @Override
    public void execute() {
       // superstructure.setDesiredState(Robot.reefMode.equals(ReefMode.CORAL) ? SuperstructureState.INTAKE : RobotState.getInstance().getAlgaePose().superstructure_state);

        if (superstructure.hasCoral) {
            startcounting = true;
            initTime = Timer.getFPGATimestamp();
        }
    }


    @Override
    public boolean isFinished() {
        if (Robot.reefMode.equals(ReefMode.CORAL)) {
            return superstructure.hasCoral && startcounting && Timer.getFPGATimestamp() - initTime > 0.5;
        }

        return false;



    }


    @Override
    public void end(boolean interrupted) {
       
            if (Robot.reefMode.equals(ReefMode.CORAL)) {  
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
            }

            else {
            superstructure.setDesiredState(SuperstructureState.HOME_ALGAE);
            }        
    }   
    
}
