package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveCommands.AutoAlignReef.ThirdPartAutoAlign;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import frc.robot.Subsystems.Vision.VisionSubsystem;


public class EjectCommand extends Command {
    Superstructure superstructure;
    double initTime;
    Drive drive;
    VisionSubsystem vision;

    public EjectCommand(Superstructure superstructure, Drive drive, VisionSubsystem vision) {
        this.superstructure = superstructure;
        this.drive = drive;
        this.vision = vision;

        if (!superstructure.desired_state.equals(SuperstructureState.L1_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L2_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L3_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L4_STOWED) && !superstructure.desired_state.equals(SuperstructureState.PROCESSOR) ) {
            //superstructure.isEjectingManually = true;
        

        addRequirements(superstructure.intake, superstructure.wrist, superstructure.pivot, superstructure.elevator);
        }

        else {
            addRequirements(superstructure.intake, superstructure.wrist, superstructure.pivot, superstructure.elevator);

        }
    }



    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        if (superstructure.current_state.equals(SuperstructureState.L1_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L1_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L2_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L2_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L3_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L3_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L4_STOWED)) {
            SmartDashboard.putBoolean("tried to set eject", true);
            superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.PROCESSOR)) {
            superstructure.setDesiredState(SuperstructureState.ALGAE_EJECT);
        }

        else if (!superstructure.desired_state.equals(SuperstructureState.L1_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L2_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L3_STOWED) && !superstructure.desired_state.equals(SuperstructureState.L4_STOWED) && !superstructure.desired_state.equals(SuperstructureState.PROCESSOR) ) {
            superstructure.isEjectingManually = true;
        }

        


    }

    @Override
    public void execute() {
        if (!superstructure.isEjectingManually) {
        if (superstructure.current_state.equals(SuperstructureState.L1_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L1_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L2_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L2_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L3_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L3_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.L4_STOWED)) {
            superstructure.setDesiredState(SuperstructureState.L4_EJECTED);
        }

        else if (superstructure.current_state.equals(SuperstructureState.PROCESSOR)) {
            superstructure.setDesiredState(SuperstructureState.ALGAE_EJECT);
        }

       
    }

     
    }


    @Override
    public boolean isFinished() {

        
      if (superstructure.isEjectingManually || superstructure.current_state.equals(SuperstructureState.L1_EJECTED)) {
        return false;
      }
    
      else {
        return superstructure.current_state.equals(SuperstructureState.L1_EJECTED) || superstructure.current_state.equals(SuperstructureState.L2_EJECTED) || superstructure.current_state.equals(SuperstructureState.L3_EJECTED) || superstructure.current_state.equals(SuperstructureState.L4_EJECTED);
      }
       
    }
     
    @Override
    public void end(boolean interrupted) {
        //if (!superstructure.isEjectingManually) {
        //superstructure.setDesiredState(SuperstructureState.HOME_UP);
        //CommandScheduler.getInstance().schedule(new ThirdPartAutoAlign(drive, vision, superstructure));
        //}
        superstructure.isEjectingManually = false;
        superstructure.hasAlgae = false;

        if (interrupted) {
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
            
        }
        superstructure.hasCoral = false;
    }
    
}
