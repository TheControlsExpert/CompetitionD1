 package frc.robot.Subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import org.jgrapht.alg.color.SmallestDegreeLastColoring;
import org.jgrapht.util.SupplierException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public WristIO wristIO;
    public ElevatorIO elevatorIO;
    public ElevatorSubsystem elevator = new ElevatorSubsystem();
    public WristSubsystem wrist = new WristSubsystem();
    public PivotSubsystem pivot = new PivotSubsystem();
    public IntakeSubsytem intake = new IntakeSubsytem();
  
    


//     Drive drive;

    public WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    public ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    
     private ManualMode manualMode = ManualMode.AUTOMATIC;

    DigitalInput sensorResetting = new DigitalInput(2);
    DigitalInput sensorIntaking = new DigitalInput(1);
     

    
    public boolean Reset_ready = false;

    public static double encoderElevator = 0;

   ArrayList<Double> listOfCurrents = new ArrayList<Double>();
    


    public SuperstructureState desired_state = SuperstructureState.HOME_UP;
    public SuperstructureState current_state = SuperstructureState.INITIAL;

    HashMap<SuperstructureState, Positions> setpointsMap = new HashMap<>();

    public boolean hasCoral = true;
    public boolean hasAlgae = false;
    public boolean isEjectingManually = false;


    public Superstructure(WristIO wristIO, ElevatorIO elevatorIO) {
        this.wristIO = wristIO;
        this.elevatorIO = elevatorIO;
       // setpointsMap.put(current_state, null)


       setpointsMap.put(SuperstructureState.HOME_UP, new Positions(0, 13, 2.5, Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty()));
       setpointsMap.put(SuperstructureState.HOME_ALGAE, new Positions(16, 13, 2.5, Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty()));
       setpointsMap.put(SuperstructureState.INTAKE, new Positions(16, 15, 3.5, Optional.empty(), Optional.empty(), Optional.of(false), Optional.empty()));
       
       setpointsMap.put(SuperstructureState.L4_STOWED, new Positions(0, 9, 61.5, Optional.empty(), Optional.empty(), Optional.of(true), Optional.empty()));
       setpointsMap.put(SuperstructureState.L4_EJECTED, new Positions(0, -1, 40, Optional.empty(), Optional.of(50.0), Optional.of(true), Optional.of(0.1)));

       setpointsMap.put(SuperstructureState.L3_STOWED, new Positions(0, 13, 29, Optional.empty(), Optional.empty(), Optional.of(true), Optional.empty()));
       setpointsMap.put(SuperstructureState.L3_EJECTED, new Positions(0, -1, 29, Optional.of(1.0), Optional.empty(), Optional.of(true), Optional.of(0.3)));

       setpointsMap.put(SuperstructureState.L2_STOWED, new Positions(0, 13, 9, Optional.empty(), Optional.empty(), Optional.of(true), Optional.empty()));
       setpointsMap.put(SuperstructureState.L2_EJECTED, new Positions(0, -1, 9, Optional.empty(), Optional.of(1.0), Optional.of(true), Optional.of(0.1)));

       setpointsMap.put(SuperstructureState.L1_STOWED, new Positions(15.5, -1, 10, Optional.empty(), Optional.empty(), Optional.of(true), Optional.empty()));
       setpointsMap.put(SuperstructureState.L1_EJECTED, new Positions(16, -1, 10, Optional.empty(), Optional.of(1.0), Optional.of(true), Optional.of(0.2)));

       setpointsMap.put(SuperstructureState.L3_ALGAE, new Positions(16, -5, 45, Optional.empty(), Optional.empty(), Optional.of(false), Optional.empty()));
       setpointsMap.put(SuperstructureState.L2_ALGAE, new Positions(16, -5, 25, Optional.empty(), Optional.empty(), Optional.of(false), Optional.empty()));

       setpointsMap.put(SuperstructureState.FIXING_WRIST, new Positions(0,0,0, Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty()));
       setpointsMap.put(SuperstructureState.FIXING_ELEVATOR,  new Positions(0,0,0, Optional.empty(), Optional.empty(), Optional.empty(), Optional.empty()));










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
    SmartDashboard.putBoolean("has Coral", hasCoral);
    SmartDashboard.putString("current state", current_state.toString());
    wristIO.updateInputs(wristInputs, manualMode);
    elevatorIO.updateInputs(elevatorInputs, manualMode);

    Reset_ready = !sensorResetting.get();


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
    // if (manualMode.equals(ManualMode.AUTOMATIC)) {
    if (!current_state.equals(desired_state) && isAtNode()) {
        current_state = desired_state;
    }

    //moving to positions





    if (desired_state.equals(SuperstructureState.L2_EJECTED) || desired_state.equals(SuperstructureState.L3_EJECTED)) {
        wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
        wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);

        if (Math.abs(wristInputs.armAngle - setpointsMap.get(desired_state).ArmAngle) < WristConstants.tolerancePivot) {
            elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        }

    }



    else if (desired_state.equals(SuperstructureState.L4_STOWED)) {
        elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        if (Math.abs(elevatorInputs.encoderRotations_L - setpointsMap.get(desired_state).elevatorEncoderRots) < ElevatorConstants.toleranceElevator) {
            wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
            wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);
        }

    }

    




    else if (desired_state.equals(SuperstructureState.FIXING_ELEVATOR) || desired_state.equals(SuperstructureState.FIXING_WRIST)) {

    }


    // else if (hasAlgae) {
    //     wristIO.setVerticalAngleSLOW(setpointsMap.get(SuperstructureState.L3_ALGAE).ArmAngle);
    //     wristIO.setWristPositionSLOW(setpointsMap.get(SuperstructureState.L3_ALGAE).wristAngle);
    //     elevatorIO.setPositionSLOW(setpointsMap.get(SuperstructureState.L3_ALGAE).elevatorEncoderRots);
    // }

    

    

    else {
        elevatorIO.setPosition(setpointsMap.get(desired_state).elevatorEncoderRots);
        wristIO.setVerticalAngle(setpointsMap.get(desired_state).ArmAngle);
        wristIO.setWristPosition(setpointsMap.get(desired_state).wristAngle);

    }




    //intake running

SmartDashboard.putBoolean("isEjectingManually", isEjectingManually);

    if (!isEjectingManually) { 

    if ((current_state.equals(SuperstructureState.L1_EJECTED) || 
        current_state.equals(SuperstructureState.L2_EJECTED)  ||
        current_state.equals(SuperstructureState.L3_EJECTED)  ||
        current_state.equals(SuperstructureState.L4_EJECTED))) {

            wristIO.setOutputOpenLoop(setpointsMap.get(current_state).outtakeSpeed.get());

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
    if(hasAlgae) {
    wristIO.setOutputOpenLoop(1);
    }

    else {
        wristIO.setOutputOpenLoop(0.5);
    }

}

//controlling 

SmartDashboard.putBoolean("intake sensor", !sensorIntaking.get());
SmartDashboard.putNumber("avg current", avg);
SmartDashboard.putBoolean("has algae", hasAlgae);


if (current_state.equals(SuperstructureState.INTAKE) &&  avg > 30) {
    hasCoral = true;

}



else if ((current_state.equals(SuperstructureState.L2_ALGAE) || current_state.equals(SuperstructureState.L3_ALGAE)) && avg > 30) {
   hasAlgae = true;
}






    
}


public void setDesiredState(SuperstructureState state) {
    if (setpointsMap.get(state).needsCoral.isPresent()) {
        if (setpointsMap.get(state).needsCoral.get().equals(hasCoral) || setpointsMap.get(state).needsCoral.get().equals(hasAlgae)) {
            this.desired_state = state;
            SmartDashboard.putBoolean("needs coral value", setpointsMap.get(state).needsCoral.get());
        }
    }

     else {

    this.desired_state = state;
     }

}

// public void setManualMode(ManualMode manualMode) {
//     this.manualMode = manualMode;
// }









    





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

    return ((offsetElevator < ElevatorConstants.toleranceElevator) && (offsetPivot < WristConstants.tolerancePivot) && (offsetWrist < WristConstants.toleranceWrist)) || (desired_state.equals(SuperstructureState.FIXING_ELEVATOR) || desired_state.equals(SuperstructureState.FIXING_WRIST));
    



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


  FIXING_ELEVATOR,
  FIXING_WRIST

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
