package frc.robot.Commands.ElevatorArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Robot.levelscore;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class CoralPrepCommand extends Command {
    Superstructure superstructure;
    CommandXboxController controller;
   public CoralPrepCommand(Superstructure superstructure, CommandXboxController controller) {
    this.superstructure = superstructure;
    this.controller = controller;
    addRequirements(superstructure.intake, superstructure.wrist, superstructure.pivot, superstructure.elevator);
   } 

   @Override
   public void execute() {

        if (Robot.CurrnetLevelPosition.equals(levelscore.Level4)) {
            superstructure.setDesiredState(SuperstructureState.L4_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(levelscore.Level3)) {
            superstructure.setDesiredState(SuperstructureState.L3_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(levelscore.Level2)) {
            superstructure.setDesiredState(SuperstructureState.L2_STOWED);
        }

        else if (Robot.CurrnetLevelPosition.equals(levelscore.Level1)) {
            superstructure.setDesiredState(SuperstructureState.L1_STOWED);
        }






       
   }


   @Override
   public boolean isFinished() {
       return controller.rightTrigger().getAsBoolean();
   }

   @Override
   public void end(boolean interrupted) {
       if (!controller.rightTrigger().getAsBoolean()) {
        superstructure.setDesiredState(SuperstructureState.HOME_UP);
        
       }


   }


   



   
    
}
