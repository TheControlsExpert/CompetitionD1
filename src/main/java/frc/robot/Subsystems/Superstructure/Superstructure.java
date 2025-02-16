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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class Superstructure extends SubsystemBase {
    WristIO wristIO;
    ElevatorIO elevatorIO;

    WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private boolean needsResetWrist;


    private SuperstructureState desired_state = SuperstructureState.HOME_UP_CORAL;
    private SuperstructureState current_state = SuperstructureState.INITIAL;
    private SuperstructureState next_state = SuperstructureState.HOME_UP_CORAL;
    private SuperstructureCommandInfo following_edge;

      private final Graph<SuperstructureState, SuperstructureCommandInfo> graph =
    new DefaultDirectedGraph<>(SuperstructureCommandInfo.class);

    private boolean hasCoral = true;
    
 

    private Alert needsResetAlert = new Alert("Wrist needs to be put in place", AlertType.kWarning);


    public Superstructure(WristIOKrakens wristIO, ElevatorIOKrakens elevatorIO) {
        this.wristIO = wristIO;
        this.elevatorIO = elevatorIO;
        wristIO.updateInputs(wristInputs);
        elevatorIO.updateInputs(elevatorInputs);
        needsResetWrist = wristInputs.sensorBoolean;


        for (var state : SuperstructureState.values()) {
          graph.addVertex(state);
        }

      //edges going home-up no coral  

      graph.addEdge(SuperstructureState.EJECT, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.INITIAL, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(false)));
      graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(false)));

      graph.addEdge(SuperstructureState.L3_ALGAE, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.L2_ALGAE, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.GROUND_INTAKE, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      


      graph.addEdge(SuperstructureState.L1_EJECTED, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.L2_EJECTED, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.L3_EJECTED, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));
      graph.addEdge(SuperstructureState.L4_EJECTED, SuperstructureState.HOME_UP_noCORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));


      //edges going for scoring

      for (int i = 11; i < 15; i++) {
          graph.addEdge(SuperstructureState.values()[i], SuperstructureState.values()[i+4], new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(true)));
          graph.addEdge(SuperstructureState.HOME_UP_CORAL, SuperstructureState.values()[i], new SuperstructureCommandInfo(i, i, i, Optional.empty(), Optional.of(true)));
          graph.addEdge(SuperstructureState.values()[i], SuperstructureState.HOME_UP_CORAL, new SuperstructureCommandInfo(i, i, i, Optional.empty(), Optional.of(true)));
      }

      //going into intermediate 

      graph.addEdge(SuperstructureState.INTAKE, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      graph.addEdge(SuperstructureState.HOME_DOWN, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(false)));
      graph.addEdge(SuperstructureState.HOME_UP_noCORAL, SuperstructureState.INTERMEDIATE, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(false)));

      //going into home up coral

      graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_UP_CORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      graph.addEdge(SuperstructureState.EJECT, SuperstructureState.HOME_UP_CORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      


      //randoms 

      graph.addEdge(SuperstructureState.HOME_UP_noCORAL, SuperstructureState.L3_ALGAE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(false)));
      graph.addEdge(SuperstructureState.HOME_UP_noCORAL, SuperstructureState.L2_ALGAE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(false)));
      graph.addEdge(SuperstructureState.HOME_UP_CORAL, SuperstructureState.RESET_WRIST, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
      graph.addEdge(SuperstructureState.RESET_WRIST, SuperstructureState.HOME_UP_CORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));

      graph.addEdge(SuperstructureState.HOME_UP_CORAL, SuperstructureState.EJECT, new SuperstructureCommandInfo(0, 0, 0, Optional.of(true), Optional.of(true)));
      graph.addEdge(SuperstructureState.GROUND_INTAKE, SuperstructureState.HOME_UP_CORAL, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(true)));
      graph.addEdge(SuperstructureState.HOME_UP_noCORAL, SuperstructureState.GROUND_INTAKE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));

      graph.addEdge(SuperstructureState.INTERMEDIATE, SuperstructureState.HOME_DOWN, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));

      graph.addEdge(SuperstructureState.HOME_DOWN, SuperstructureState.INTAKE, new SuperstructureCommandInfo(0, 0, 0, Optional.of(false), Optional.of(false)));


      //transition between scoring stows


      for (int i = 11; i<15; i++) {

          for (int j = 11; j<15; j++) {
              if (i!=j) {
      graph.addEdge(SuperstructureState.values()[i], SuperstructureState.values()[j], new SuperstructureCommandInfo(0, 0, 0, Optional.empty(), Optional.of(true)));
              }
          }
      }


        



    

    }


    @Override
    public void periodic() {

        wristIO.updateInputs(wristInputs);
        elevatorIO.updateInputs(elevatorInputs);

        needsResetAlert.set(needsResetWrist);

        //are we scoring

        if (isAtNode()) {
            current_state = next_state;
            if (!current_state.equals(desired_state)) {
                next_state = runSearch(current_state, desired_state).get();
                following_edge = graph.getEdge(current_state, next_state);

                //controlling code

                

            }
          }


        else {
          //controlling code

          if (following_edge.elevatorFirst.isPresent()) {

            //elevator has to move first
            if (following_edge.elevatorFirst.get()) {
               elevatorIO.setPosition(following_edge.elevatorEncoderRots);
              
              if (Math.abs(following_edge.elevatorEncoderRots - elevatorInputs.encoderRotations_L) < ElevatorConstants.toleranceElevator) {
               wristIO.setVerticalAngle(following_edge.pivotPos);
               wristIO.setWristPosition(following_edge.wristAngle);
              }

            }

            //arm has to move first

            else {
              wristIO.setVerticalAngle(following_edge.pivotPos);
              wristIO.setWristPosition(following_edge.wristAngle);

              if (Math.abs(following_edge.pivotPos - wristInputs.armAngle) < WristConstants.tolerancePivot) {
                elevatorIO.setPosition(following_edge.elevatorEncoderRots);
              }

            }



          }

          else {

            //move at the same time
            
            wristIO.setVerticalAngle(following_edge.pivotPos);
            wristIO.setWristPosition(following_edge.wristAngle);
            elevatorIO.setPosition(following_edge.elevatorEncoderRots);

          }


        }


        if (current_state.equals(SuperstructureState.INTAKE) && wristInputs.current > IntakeConstants.currentMax) {
          hasCoral = true;
        }

        if (current_state.equals(SuperstructureState.EJECT) | 
            current_state.equals(SuperstructureState.L1_EJECTED) |
            current_state.equals(SuperstructureState.L2_EJECTED) |
            current_state.equals(SuperstructureState.L3_EJECTED) |
            current_state.equals(SuperstructureState.L4_EJECTED)) {
          hasCoral = false;
            }
      
            
        //add logic for manual mode  


   
   



        
        

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
        .filter(edge -> isEdgeAllowed(edge))
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
    this.desired_state = state;
}

public boolean hasCoral() {
    return hasCoral;
}


public boolean isEdgeAllowed(SuperstructureCommandInfo edge) {
  //can go on path bcs we have coral
  return edge.requiresCoral.get().equals(hasCoral);

}


public boolean isAtNode() {
  double offsetElevator =  Math.abs(graph.getEdge(current_state, next_state).elevatorEncoderRots - elevatorInputs.encoderRotations_L);
  double offsetPivot = Math.abs(graph.getEdge(current_state, next_state).pivotPos - wristInputs.armAngle);
  double offsetWrist = Math.abs(graph.getEdge(desired_state, current_state).wristAngle - wristInputs.wristAngle);

  return ((offsetElevator < ElevatorConstants.toleranceElevator) && (offsetPivot < WristConstants.tolerancePivot) && (offsetWrist < WristConstants.toleranceWrist));
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
  HOME_UP_noCORAL,
  HOME_UP_CORAL,
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
