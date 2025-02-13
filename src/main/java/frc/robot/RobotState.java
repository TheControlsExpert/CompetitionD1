package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer.ScoringLevel;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;

public class RobotState {

  public static RobotState instance;
    
  public static ScoringLevel currentScoringLevel = ScoringLevel.L4;
  public static ScoringPosition currentScoringCommand = ScoringPosition.A;

  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter = new HashMap<>();
  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter_offset = new HashMap<>();


  public RobotState() {
    //do all Hashmap stuff
  }




  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }


  public Pose2d[] getScoringPose() {
    return PositionGetter.get(currentScoringCommand);
  }

  public Pose2d[] getScoringPose_offset() {
    return PositionGetter_offset.get(currentScoringCommand);
  }


  public Pose2d getIntakingPose(Pose2d currentPose) {

    if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      double distanceR = currentPose.getTranslation().minus(FieldConstants.RightSource_BLUE.getTranslation()).getNorm();
      double distanceL = currentPose.getTranslation().minus(FieldConstants.LeftSource_BLUE.getTranslation()).getNorm();

      if (distanceL >= distanceR) {
        //since we are closer to the right source, go there
        return FieldConstants.RightSource_BLUE;
      }

      else {
        return FieldConstants.LeftSource_BLUE;
      }

    }

    
    else {
      double distanceR = currentPose.getTranslation().minus(FieldConstants.RightSource_RED.getTranslation()).getNorm();
      double distanceL = currentPose.getTranslation().minus(FieldConstants.LeftSource_RED.getTranslation()).getNorm();

      if (distanceL >= distanceR) {
        //since we are closer to the right source, go there
        return FieldConstants.RightSource_RED;
      }

      else {
        return FieldConstants.LeftSource_RED;
      }

    }
  }

    
}
