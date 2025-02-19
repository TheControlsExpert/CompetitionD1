package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class IntakeCommandArm extends Command {
   
    private final Superstructure superstructure;

    boolean startCounting = false;
    double initTime = 0;
   
    

    public IntakeCommandArm(Superstructure superstructure) {
        
        this.superstructure = superstructure;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.INTAKE);
    }

    @Override
    public void execute() {
      
    }

    @Override
    public boolean isFinished() {
        return superstructure.hasCoral() || Timer.getFPGATimestamp() - initTime > 1.5;
       

        
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
        

    
}

}
