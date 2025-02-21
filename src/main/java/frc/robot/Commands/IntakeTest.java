package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class IntakeTest extends Command {

    private final Superstructure superstructure;
    private double timeInitial;
    private double maxCurrent = 0;
    public IntakeTest(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        timeInitial = Timer.getFPGATimestamp();
        superstructure.setIntakeManual(-0.2);
    }

    @Override
    public void execute() {
        if (superstructure.getCurrentIntake() > maxCurrent) {
            maxCurrent = superstructure.getCurrentIntake();
        }

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - timeInitial > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setIntakeManual(0);
        if (maxCurrent > 0.5) {
        superstructure.hasCoral = true;
    }
    else {
        superstructure.hasCoral = false;
    }

    superstructure.hasDoneIntakeTest = true;
    }
    
}
