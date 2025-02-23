package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class IntakeTest extends Command {

    private final Superstructure superstructure;
    private double timeInitial;
    private double currentAvg = 0;
    private int numCount  = 0;


    public IntakeTest(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
        
    }

    @Override
    public void initialize() {
        currentAvg = 0;
        numCount = 0;
        timeInitial = Timer.getFPGATimestamp();
        superstructure.setIntakeManual(-0.2);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("maxcurrent", currentAvg);
         currentAvg = (currentAvg * numCount + superstructure.getCurrentIntake()) / (numCount + 1);
        numCount++;

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timeInitial > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setIntakeManual(0);
        if (currentAvg > 60) {
        superstructure.hasCoral = true;
    }
    else {
        superstructure.hasCoral = false;
    }

    superstructure.hasDoneIntakeTest = true;
    }
    
}
