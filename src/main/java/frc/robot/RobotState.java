package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class RobotState {

  public static RobotState instance;
    
  public static Superstructure.SuperstructureState currentScoringLevel = Superstructure.SuperstructureState.L1_STOWED;
  public static ScoringPosition currentScoringCommand = ScoringPosition.A;
  public double fieldMiddleX = 8.774176;
  public double fieldMiddleY = 4.03352;

  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter = new HashMap<>();
  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter_offset = new HashMap<>();
  public static final double poseXOffCenterA = 0;
  public static final double poseYOffCenterA = 0.33/2;
  public static final double poseThetaOffCenterA = Math.atan(poseXOffCenterA / poseYOffCenterA);
  public static final double RadiusHexagon = Math.hypot(poseXOffCenterA, poseYOffCenterA);
  public static final Pose2d poseCenterReefBlue = new Pose2d(0, 0, new Rotation2d(0));




  


  public RobotState() {
    // //do all Hashmap stuff
    Pose2d scoringPoseABLUE = new Pose2d(new Translation2d(Math.cos( Math.PI - poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI - poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0));
    Pose2d[] As = {scoringPoseABLUE, getRedPose(scoringPoseABLUE)};
    PositionGetter.put(ScoringPosition.A, As);


    Pose2d scoringPoseBBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI + poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0));
    Pose2d[] Bs = {scoringPoseABLUE, getRedPose(scoringPoseABLUE)};
    PositionGetter.put(ScoringPosition.B, Bs);

    Pose2d scoringPoseCBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI + poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0));





  }




  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private Pose2d getRedPose(Pose2d pose) {
    double x =  -pose.getTranslation().getX() + fieldMiddleX * 2;
    double y = -pose.getTranslation().getY() + fieldMiddleY * 2;

    return new Pose2d(x, y, pose.getRotation().plus(new Rotation2d(Math.PI)));



  }


  public Pose2d[] getScoringPose() {
    return PositionGetter.get(currentScoringCommand);
  }

  public Pose2d[] getScoringPose_offset() {
    return PositionGetter_offset.get(currentScoringCommand);
  }

  public boolean isRobotFarEnough(Pose2d currentPose) {
    Pose2d scoringPose = DriverStation.getAlliance().get() == Alliance.Blue ? getScoringPose()[0] : getScoringPose()[1];

    return currentPose.getTranslation().minus(scoringPose.getTranslation()).getNorm() > 1;

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
