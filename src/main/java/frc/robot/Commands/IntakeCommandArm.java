package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class IntakeCommandArm extends Command {
   
    private final Superstructure superstructure;
    private final Intake intake;

    boolean startCounting = false;
    double initTime = 0;
   
    

    public IntakeCommandArm(Superstructure superstructure, Intake intake) {
        
        this.superstructure = superstructure;
        this.intake = intake;

        addRequirements(superstructure);
    }

    

    @Override
    public void execute() {
        if (intake.CurrentintakeState.equals(Intake.Intake_states.Ready) && !startCounting) {
            superstructure.setDesiredState(SuperstructureState.INTAKE);
            startCounting = true;
            initTime = Timer.getFPGATimestamp();
        }
      
    }

    @Override
    public boolean isFinished() {
        return (superstructure.hasCoral() && Timer.getFPGATimestamp() - initTime > 1 && startCounting) || (Timer.getFPGATimestamp() - initTime > 1.5 && startCounting);
        
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);     
}

}
