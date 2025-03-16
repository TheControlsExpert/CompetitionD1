package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;


public class ResetElevatorCommand extends Command {
    Superstructure superstructure;
    double initTime = -10000;

    public ResetElevatorCommand(Superstructure superstructure) {
        addRequirements(superstructure.elevator, superstructure.pivot, superstructure.wrist, superstructure.intake);
        this.superstructure = superstructure;
       
    }


    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.FIXING_ELEVATOR);
        initTime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        superstructure.setElevatorManual(-0.05);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("time number passed", Timer.getFPGATimestamp() - initTime);
        return Timer.getFPGATimestamp() - initTime > 0.2 && Math.abs(superstructure.elevatorIO.getEncoderSpeed()) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
    superstructure.setElevatorManual(0);
    
    superstructure.elevatorIO.resetPosition();
    
    superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }


    
}
