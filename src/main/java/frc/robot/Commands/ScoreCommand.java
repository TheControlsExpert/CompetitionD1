package frc.robot.Commands;

import org.jgrapht.alg.color.SmallestDegreeLastColoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class ScoreCommand extends Command {
    double initTime;
    boolean starttimer = false;
    private final Superstructure superstructure;
    public ScoreCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }


    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        //superstructure.hasCoral = false;

        SmartDashboard.putBoolean("scoring called", true);
    }

    @Override
    public void execute() {
        if (superstructure.getCurrentState().equals(SuperstructureState.L4_EJECTED)) {
            starttimer = true;
            initTime = Timer.getFPGATimestamp();
           
            //superstructure.hasCoral = false;
        }

        if (starttimer && Timer.getFPGATimestamp() - initTime > 0.5) {
            superstructure.hasCoral = false;
        }
    }

    @Override
    public boolean isFinished() {
        return !superstructure.hasCoral() && Timer.getFPGATimestamp() - initTime > 0.5;

    }


    @Override
    public void end(boolean interrupted) {
        
       
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }

    
    
}
