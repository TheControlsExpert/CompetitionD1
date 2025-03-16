 package frc.robot.Subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import org.jgrapht.util.SupplierException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

// import static edu.wpi.first.units.Units.derive;

// import java.util.Arrays;
// import java.util.HashMap;
// import java.util.LinkedList;
// import java.util.Map;
// import java.util.Optional;
// import java.util.Queue;

// import org.jgrapht.Graph;
// import org.jgrapht.graph.DefaultDirectedGraph;
// import org.jgrapht.graph.DefaultEdge;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotState;
// import frc.robot.Commands.IntakeTest;
// import frc.robot.Subsystems.Drive.Drive;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.WristConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class Superstructure extends SubsystemBase {
    WristIO wristIO;
    ElevatorIO elevatorIO;
    public ElevatorSubsystem elevator = new ElevatorSubsystem();
    public WristSubsystem wrist = new WristSubsystem();
    public PivotSubsystem pivot = new PivotSubsystem();
    public IntakeSubsytem intake = new IntakeSubsytem();
  
    


//     Drive drive;

    WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    
     private ManualMode manualMode = ManualMode.AUTOMATIC;
     

    private boolean hasScheduledYet = false;
    public boolean hasDoneIntakeTest = false;

    public static double encoderElevator = 0;

   ArrayList<Double> listOfCurrents = new ArrayList<Double>();
    


    public SuperstructureState desired_state = SuperstructureState.HOME_UP;
    public SuperstructureState current_state = SuperstructureState.INITIAL;

    HashMap<SuperstructureState, Positions> setpointsMap = new HashMap<>();

    public boolean hasCoral = true;
    public boolean hasAlgae = false;
    public boolean isEjectingManually = true;


    public Superstructure(WristIO wristIO, ElevatorIO elevatorIO) {
        this.wristIO = wristIO;
        this.elevatorIO = elevatorIO;
       // setpointsMap.put(current_state, null)
    }





      
      


     
    




public SuperstructureState getCurrentState() {
  return current_state;
 }

public ManualMode getManualMode() {
  return manualMode;
}

// public double getCurrentIntake() {
//   return wristInputs.current;
// }




public static enum ManualMode {
  MANUAL,
  AUTOMATIC
}


@Override
public void periodic() {
    wristIO.updateInputs(wristInputs, manualMode);
    elevatorIO.updateInputs(elevatorInputs, manualMode);


    double avg = 0;
    if (!listOfCurrents.isEmpty()) {
    for (double d : listOfCurrents) {
      avg += d;
    }
    avg = avg / listOfCurrents.size();
    }
    
    
    listOfCurrents.add(wristInputs.current);
    if (listOfCurrents.size() > 20) {
      listOfCurrents.remove(0);  
    }


    //state transitions
    if (manualMode.equals(ManualMode.AUTOMATIC)) {
    if (!current_state.equals(desired_state) && isAtNode()) {
        current_state = desired_state;
    }

    //moving to positions





    if (desired_state.equals(SuperstructureState.HOME_UP) || desired_state.equals(SuperstructureState.L2_EJECTED) || desired_state.equals(SuperstructureState.L3_EJECTED)) {
        wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
        wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);

        if (Math.abs(wristInputs.armAngle - setpointsMap.get(desired_state).ArmAngle) < WristConstants.tolerancePivot) {
            elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        }

    }

    else if (desired_state.equals(SuperstructureState.L4_STOWED)) {
        elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        if (elevatorInputs.encoderRotations_L > ElevatorConstants.minHeightforL4Pivot) {
            wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
            wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);
        }

    }

    




    else if (current_state.equals(SuperstructureState.HOME_UP)) {
        elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        if (elevatorInputs.encoderRotations_L > ElevatorConstants.minHeightAboveHome) {
            wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
            wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);
        }


    }

    

    

    else {
        elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
        wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);

    }




    //intake running



    if (!isEjectingManually) { 

    if ((desired_state.equals(SuperstructureState.L1_EJECTED) || 
        desired_state.equals(SuperstructureState.L2_EJECTED)  ||
        desired_state.equals(SuperstructureState.L3_EJECTED)  ||
        desired_state.equals(SuperstructureState.L4_EJECTED)) && ((setpointsMap.get(desired_state).ElevatorHeightForRelease.isPresent() && setpointsMap.get(desired_state).ElevatorHeightForRelease.get() > elevatorInputs.encoderRotations_L) || (setpointsMap.get(desired_state).PivotAngleForRelease.isPresent() && setpointsMap.get(desired_state).PivotAngleForRelease.get() > wristInputs.armAngle))) {

            wristIO.setOutputOpenLoop(setpointsMap.get(desired_state).outtakeSpeed.get());

            //ready to eject

        }
    else if (current_state.equals(SuperstructureState.INTAKE) && !hasCoral) {
        wristIO.setOutputOpenLoop(-1);
    }

    else if (hasCoral) {
        wristIO.keepStill();
    }

    else if ((current_state.equals(SuperstructureState.L2_ALGAE) || current_state.equals(SuperstructureState.L3_ALGAE)) && !hasAlgae) {
        wristIO.setOutputOpenLoop(-0.5);
    }

    else if (hasAlgae) {
        wristIO.keepStill();
    }

    else {
        wristIO.setOutputOpenLoop(0);
    }



}

else {
    wristIO.setOutputOpenLoop(0.5);

}

//controlling 


if (current_state.equals(SuperstructureState.INTAKE) && avg > 30) {
    hasCoral = true;

}

else if (current_state.equals(SuperstructureState.L2_ALGAE) || current_state.equals(SuperstructureState.L3_ALGAE) && avg > 30) {
   hasAlgae = true;
}

}


    
}


public void setDesiredState(SuperstructureState state) {
    if (setpointsMap.get(state).needsCoral.isPresent()) {
        if (setpointsMap.get(state).needsCoral.get().equals(hasCoral)) {
            this.desired_state = state;
        }
    }

}

public void setManualMode(ManualMode manualMode) {
    this.manualMode = manualMode;
}









    





// public boolean hasCoral() {
//     return hasCoral;
// }


// public boolean isEdgeAllowed(SuperstructureCommandInfo edge, SuperstructureState state) {
//   //can go on path bcs we have coral
//   SmartDashboard.putBoolean("is edge allowed", false);
 

//   return edge.requiresCoral.get().equals(hasCoral) || state.equals(SuperstructureState.HOME_UP);
  

// }


public boolean isAtNode() {
    double offsetElevator =  Math.abs(setpointsMap.get(desired_state).elevatorEncoderRots - elevatorInputs.encoderRotations_L);
    double offsetPivot = Math.abs(setpointsMap.get(desired_state).ArmAngle - wristInputs.armAngle);
    double offsetWrist = Math.abs(setpointsMap.get(desired_state).wristAngle - wristInputs.wristAngle);

    return ((offsetElevator < ElevatorConstants.toleranceElevator) && (offsetPivot < WristConstants.tolerancePivot) && (offsetWrist < WristConstants.toleranceWrist));
    

}





 public static enum SuperstructureState {

  L2_ALGAE,
  L3_ALGAE,
  PROCESSOR,
  HOME_ALGAE,
  ALGAE_EJECT,

  INITIAL,

  HOME_UP,
  HOME_DOWN,

  INTAKE,

  L1_STOWED,
  L2_STOWED,
  L3_STOWED,
  L4_STOWED,


  L1_EJECTED,
  L2_EJECTED,
  L3_EJECTED,
  L4_EJECTED,

}

public static record Positions(double wristAngle, double ArmAngle, double elevatorEncoderRots, Optional<Double> ElevatorHeightForRelease, Optional<Double> PivotAngleForRelease,  Optional<Boolean> needsCoral, Optional<Double> outtakeSpeed) {

}



// //manual commands

public void setWristManual(double output) {
  wristIO.setOutputOpenLoopWrist(output);

}

public void setElevatorManual(double output) {
  elevatorIO.setOutputOpenLoop(output);
}

public void setPivotManual(double output) {
  wristIO.setOutputOpenLoopPivot(output);
}

public void setIntakeManual(double output) {
  wristIO.setOutputOpenLoop(output);
}

}
