package frc.robot.Subsystems.Superstructure;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class Superstructure extends SubsystemBase {
    WristIO wristIO;
    ElevatorIO elevatorIO;
    Drive drive;

    WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private boolean needsResetWrist;


    private SuperstructureState desired_state = SuperstructureState.HOME_UP;
    private SuperstructureState current_state = SuperstructureState.INITIAL;
    private SuperstructureState next_state = SuperstructureState.HOME_UP;
    private SuperstructureState requestedState = SuperstructureState.HOME_UP;
    private SuperstructureCommandInfo following_edge;
      private final Graph<SuperstructureState, SuperstructureCommandInfo> graph =
    new DefaultDirectedGraph<>(SuperstructureCommandInfo.class);

    private boolean hasCoral = true;
    
 

    private Alert needsResetAlert = new Alert("Wrist needs to be put in place", AlertType.kWarning);


    public Superstructure(WristIOKrakens wristIO, ElevatorIOKrakens elevatorIO, Drive drive) {
        this.drive = drive;
        this.wristIO = wristIO;
        this.elevatorIO = elevatorIO;
        wristIO.updateInputs(wristInputs);
        elevatorIO.updateInputs(elevatorInputs);
        needsResetWrist = wristInputs.sensorBoolean;


        for (var state : SuperstructureState.values()) {
          graph.addVertex(state);
        }

      //edges going home-up no coral  

      graph.addEdge(SuperstructureState.EJECT, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.INITIAL, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(17.5, 2, 0, Optional.empty(), Optional.of(false)));
      graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(17.5, 2, 0, Optional.empty(), Optional.of(false)));

     // graph.addEdge(SuperstructureState.L3_ALGAE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
     // graph.addEdge(SuperstructureState.L2_ALGAE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
    //  graph.addEdge(SuperstructureState.GROUND_INTAKE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      


     // graph.addEdge(SuperstructureState.L1_EJECTED, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      //graph.addEdge(SuperstructureState.L2_EJECTED, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
     // graph.addEdge(SuperstructureState.L3_EJECTED, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.L4_EJECTED, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(17.5, 2, 0, Optional.of(false), Optional.of(false)));


      //edges going for scoring

      // for (int i = 11; i < 15; i++) {
      //     graph.addEdge(SuperstructureState.values()[i], SuperstructureState.values()[i+4], new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(true)));
      //     graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.values()[i], new SuperstructureCommandInfo(i, i, i, Optional.empty(), Optional.of(true)));
      //     graph.addEdge(SuperstructureState.values()[i], SuperstructureState.HOME_UP, new SuperstructureCommandInfo(i, i, i, Optional.empty(), Optional.of(true)));
      // }

      graph.addEdge(SuperstructureState.L4_STOWED, SuperstructureState.L4_EJECTED, new SuperstructureCommandInfo(11, 55, 0, Optional.of(false), Optional.of(true)));
      graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.L4_STOWED, new SuperstructureCommandInfo(17.5, 62.5, 0, Optional.empty(), Optional.of(true)));
      graph.addEdge(SuperstructureState.L4_STOWED, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(17.5, 2, 0, Optional.empty(), Optional.of(true)));

      //going into intermediate 



      graph.addEdge(SuperstructureState.INTAKE, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(-18, 41, 0, Optional.empty(), Optional.of(true)));
      graph.addEdge(SuperstructureState.HOME_DOWN, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(-18, 41, 0, Optional.empty(), Optional.of(false)));
      graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(17.5, 41, 0, Optional.empty(), Optional.of(false)));

      //going into home up coral

     // graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
     // graph.addEdge(SuperstructureState.EJECT, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      


      //randoms 

      //graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.L3_ALGAE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(false)));
     // graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.L2_ALGAE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(false)));
      //graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.RESET_WRIST, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      //graph.addEdge(SuperstructureState.RESET_WRIST, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));

     // graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.EJECT, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(true)));
    //  graph.addEdge(SuperstructureState.GROUND_INTAKE, SuperstructureState.HOME_UP, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(true)));
     // graph.addEdge(SuperstructureState.HOME_UP, SuperstructureState.GROUND_INTAKE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));

      graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_DOWN, new SuperstructureCommandInfo(-17.5, 27, 0, Optional.of(false), Optional.of(false)));

      graph.addEdge(SuperstructureState.HOME_DOWN, SuperstructureState.INTAKE, new SuperstructureCommandInfo(-18, 16.75, 0, Optional.of(false), Optional.of(false)));


      //transition between scoring stows


      // for (int i = 11; i<15; i++) {

      //     for (int j = 11; j<15; j++) {
      //         if (i!=j) {
      // graph.addEdge(SuperstructureState.values()[i], SuperstructureState.values()[j], new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      //         }
      //     }
      // }


        


    following_edge = graph.getEdge(current_state, desired_state);
    

    }


    @Override
    public void periodic() {
      SmartDashboard.putNumber("setpoint elevator", following_edge.elevatorEncoderRots);
      SmartDashboard.putNumber("setpoint pivot", following_edge.pivotPos);
      SmartDashboard.putString("following edge", graph.getEdgeTarget(following_edge).toString());
      SmartDashboard.putString("current state", current_state.toString());
      SmartDashboard.putString("desired state", desired_state.toString());
      SmartDashboard.putString("next state", next_state.toString());



     

        wristIO.updateInputs(wristInputs);
        elevatorIO.updateInputs(elevatorInputs);

        needsResetAlert.set(needsResetWrist);

        //are we scoring

        if (isAtNode()) {
        
          
            current_state = next_state;
            desired_state = requestedState;
            //SmartDashboard.putString("requested state", requestedState.toString());
            if (!current_state.equals(desired_state)) {
                next_state = runSearch(current_state, desired_state).get();
                following_edge = graph.getEdge(current_state, next_state);

                //controlling code

                

            }
          }

        else {
          //controlling code

          //make sure we are far enough from reef (if necessary)

         // if ((current_state.equals(SuperstructureState.L1_EJECTED) || current_state.equals(SuperstructureState.L2_EJECTED) || current_state.equals(SuperstructureState.L3_EJECTED) || current_state.equals(SuperstructureState.L4_EJECTED) || current_state.equals(SuperstructureState.L2_ALGAE) || current_state.equals(SuperstructureState.L3_ALGAE)) && graph.getEdgeTarget(following_edge).equals(SuperstructureState.HOME_UP) && RobotState.getInstance().isRobotFarEnough(drive.getEstimatedPosition())) {
           
            
          

          if (following_edge.elevatorFirst.isPresent()) {

            //elevator has to move first
            if (following_edge.elevatorFirst.get()) {
               elevatorIO.setPosition(following_edge.elevatorEncoderRots);
              
              if (Math.abs(following_edge.elevatorEncoderRots - elevatorInputs.encoderRotations_L) < ElevatorConstants.toleranceElevator) {
               wristIO.setVerticalAngle(following_edge.pivotPos);
               //wristIO.setWristPosition(following_edge.wristAngle);
              }

            }

            //arm has to move first

            else {
             
              wristIO.setVerticalAngle(following_edge.pivotPos);
              //wristIO.setWristPosition(following_edge.wristAngle);

              if (Math.abs(following_edge.pivotPos - wristInputs.armAngle) < WristConstants.tolerancePivot) {
                elevatorIO.setPosition(following_edge.elevatorEncoderRots);
              }

            }



          }

          else {

            //move at the same time

            if (!next_state.equals(SuperstructureState.INTERMEDIATE)) {
            
            wristIO.setVerticalAngle(following_edge.pivotPos);
            //wristIO.setWristPosition(following_edge.wristAngle);
            elevatorIO.setPosition(following_edge.elevatorEncoderRots);
            }

            else {
              elevatorIO.setPosition(following_edge.elevatorEncoderRots);
            }

          }


        }
      

        //hasCoral logic
        SmartDashboard.putBoolean("does really have coral", hasCoral);

        if (wristInputs.current > 45) {
          hasCoral = true;
        }

        else {
          hasCoral = false;
        }

        //check to see if we are in a state that needs coral

        // graph.edgesOf(current_state).stream().filter((command) -> )  (current_state);


        // if ()

        



      
            
        //add intaking/outtaking stuff here

        if (current_state.equals(SuperstructureState.INTAKE) || current_state.equals(SuperstructureState.GROUND_INTAKE) && !hasCoral) {
          wristIO.setOutputOpenLoop(-0.2);
          
        }


        else if (current_state.equals(SuperstructureState.EJECT)) {
          wristIO.setOutputOpenLoop(0.2);
        }

        else if ((current_state.equals(SuperstructureState.L1_EJECTED) || current_state.equals(SuperstructureState.L2_EJECTED) || current_state.equals(SuperstructureState.L3_EJECTED) || current_state.equals(SuperstructureState.L4_EJECTED)) 
        ) {       
          wristIO.setOutputOpenLoop(0.2);
        }

        else if ((current_state.equals(SuperstructureState.L2_ALGAE) | current_state.equals(SuperstructureState.L3_ALGAE))  
        ) {

        }

        else if (hasCoral) {
          wristIO.setOutputOpenLoop(-0.2);
        }

        
        else {
          wristIO.setOutputOpenLoop(0);
        }




}






private Optional<SuperstructureState> runSearch(SuperstructureState start, SuperstructureState goal) {

  double initTime = System.nanoTime();

// Map to track the parent of each visited node
Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
Queue<SuperstructureState> queue = new LinkedList<>();
queue.add(start);
parents.put(start, null); // Mark the start node as visited with no parent
// Perform BFS
while (!queue.isEmpty()) {
SuperstructureState current = queue.poll();
// Check if we've reached the goal
if (current.equals(goal)) {
  break;
}
// Process valid neighbors
for (SuperstructureCommandInfo edge :
    graph.outgoingEdgesOf(current).stream()
        .filter(edge -> isEdgeAllowed(edge, goal))
        .toList()) {
  SuperstructureState neighbor = graph.getEdgeTarget(edge);
  // Only process unvisited neighbors
  if (!parents.containsKey(neighbor)) {
    parents.put(neighbor, current);
    queue.add(neighbor);
  }
}
}

// Reconstruct the path to the goal if found
if (!parents.containsKey(goal)) {
System.out.println((System.nanoTime() - initTime) / 1000000);
return Optional.empty(); // Goal not reachable
}

// Trace back the path from goal to start
SuperstructureState nextState = goal;
while (!nextState.equals(start)) {
SuperstructureState parent = parents.get(nextState);
if (parent == null) {
  return Optional.empty(); // No valid path found
} else if (parent.equals(start)) {
  // Return the edge from start to the next node
  System.out.println((System.nanoTime() - initTime) / 1000000);
  return Optional.of(nextState);
}
nextState = parent;
}

System.out.println((System.nanoTime() - initTime) / 1000000);
return Optional.of(nextState);
} 



// public enum stateGroup {
//     UP,
//     DOWN
// }





// public record AngleStates(
//     double wristAngle,
//     double armAngle,
//     double elevatorHeight) {
// }  


   
    



public void setDesiredState(SuperstructureState state) {
    if (runSearch(next_state, state).isPresent()) {
    requestedState = state;
    SmartDashboard.putBoolean("edge was present", true);
    SmartDashboard.putBoolean("has coral", hasCoral);
    SmartDashboard.putString("requested state", requestedState.toString());
}

else {
    SmartDashboard.putBoolean("edge was present", false);
    SmartDashboard.putString("requested state", requestedState.toString());
}
}

public boolean hasCoral() {
    return hasCoral;
}


public boolean isEdgeAllowed(SuperstructureCommandInfo edge, SuperstructureState state) {
  //can go on path bcs we have coral
 

  return edge.requiresCoral.get().equals(hasCoral) || state.equals(SuperstructureState.HOME_UP);
  

}


public boolean isAtNode() {
  if (current_state != next_state) {
  double offsetElevator =  Math.abs(graph.getEdge(current_state, next_state).elevatorEncoderRots - elevatorInputs.encoderRotations_L);
  double offsetPivot = Math.abs(graph.getEdge(current_state, next_state).pivotPos - wristInputs.armAngle);
  //double offsetWrist = Math.abs(graph.getEdge(desired_state, current_state).wristAngle - wristInputs.wristAngle);

  SmartDashboard.putNumber("offset pivot", offsetPivot);
  SmartDashboard.putNumber("offset elevator", offsetElevator);
  SmartDashboard.putNumber("offset pivot edge state", graph.getEdge(current_state, next_state).pivotPos);
  if (!next_state.equals(SuperstructureState.INTERMEDIATE)) {

  return ((offsetElevator < ElevatorConstants.toleranceElevator) && (offsetPivot < WristConstants.tolerancePivot));
  }

  else {
    SmartDashboard.putBoolean("close enough to elevator setpoint", offsetElevator < ElevatorConstants.toleranceElevator);
    return offsetElevator < ElevatorConstants.toleranceElevator;
  }
}
return true;
}



public static class SuperstructureCommandInfo extends DefaultEdge {
  double pivotPos;
double elevatorEncoderRots;
double wristAngle;
Optional<Boolean> elevatorFirst;
Optional<Boolean> requiresCoral;

public SuperstructureCommandInfo(double pivotPos,
double elevatorEncoderRots,
double wristAngle,
Optional<Boolean> elevatorFirst,
Optional<Boolean> requiresCoral) {
  super();

  this.elevatorEncoderRots = elevatorEncoderRots;
  this.elevatorFirst = elevatorFirst;
  this.pivotPos = pivotPos;
  this.requiresCoral = requiresCoral;
  this.wristAngle = wristAngle;
 

}
}

public static enum SuperstructureState {
  L3_ALGAE,
  L2_ALGAE,
  INITIAL,
  HOME_UP,
  
  HOME_DOWN,
  INTERMEDIATE,
  GROUND_INTAKE,
  EJECT,
  INTAKE,
  RESET_WRIST,


  L1_STOWED,
  L2_STOWED,
  L3_STOWED,
  L4_STOWED,


  L1_EJECTED,
  L2_EJECTED,
  L3_EJECTED,
  L4_EJECTED
}



}
