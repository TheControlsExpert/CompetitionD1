package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.Intake_states;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import pabeles.concurrency.IntRangeTask;

public class IntakeCommandArm extends Command {
   
    private final Superstructure superstructure;
    private final Intake intake;

    boolean startCounting = false;
    double initTime = 0;
    double actualiniTime = 0;
   
    

    public IntakeCommandArm(Superstructure superstructure, Intake intake) {
        
        this.superstructure = superstructure;
        this.intake = intake;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        actualiniTime = Timer.getFPGATimestamp();
    }

    

    @Override
    public void execute() {
        if (intake.CurrentintakeState.equals(Intake.Intake_states.Ready) && !startCounting && superstructure.getCurrentState().equals(SuperstructureState.HOME_DOWN)) {
            superstructure.setDesiredState(SuperstructureState.INTAKE);
            
        }

        if (superstructure.getCurrentState().equals(Superstructure.SuperstructureState.INTAKE)) {
            startCounting = true;
            initTime = Timer.getFPGATimestamp();
        }
      
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("is timing out", Timer.getFPGATimestamp() - initTime > 2 && startCounting);
        return (intake.CurrentintakeState.equals(Intake_states.Empty) && Timer.getFPGATimestamp() - initTime > 0.25 && startCounting) || (Timer.getFPGATimestamp() - actualiniTime > 3);
        
    }

    @Override
    public void end(boolean interrupted) {
        if (intake.CurrentintakeState.equals(Intake_states.Empty)) {
            superstructure.hasCoral = true;
        }

        else {
            superstructure.hasCoral = false;
            intake.setState(Intake_states.Empty);
        }
        superstructure.setDesiredState(SuperstructureState.HOME_UP);     
}

}
