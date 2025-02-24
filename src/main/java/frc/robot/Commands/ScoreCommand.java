package frc.robot.Commands;

import org.jgrapht.alg.color.SmallestDegreeLastColoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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

         if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level4)) {
            superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level3)) {
            superstructure.setDesiredState(SuperstructureState.L3_EJECTED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level2)) {
            superstructure.setDesiredState(SuperstructureState.L2_EJECTED);
        }

        else if (Robot.CurrnetLevelPosition.equals(Robot.levelscore.Level1)) {
            superstructure.setDesiredState(SuperstructureState.L1_EJECTED);
        }


       
        //superstructure.hasCoral = false;

        SmartDashboard.putBoolean("scoring called", true);
    }

   

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - initTime > 1.5;

    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("scoring somehow ended", true);
        superstructure.hasCoral = false;
        
       
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
    }

    
    
}
