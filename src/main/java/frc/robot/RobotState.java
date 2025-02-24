package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class RobotState {

  public static RobotState instance;
    
  public static Superstructure.SuperstructureState currentScoringLevel = Superstructure.SuperstructureState.L1_STOWED;
  public static ScoringPosition currentScoringCommand = ScoringPosition.A;
  public double fieldMiddleX = 8.774176;
  public double fieldMiddleY = 4.0259;

  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter = new HashMap<>();
  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter_offset = new HashMap<>();
  public static final double poseXOffCenterA = 3.241098;
  public static final double poseYOffCenterA = 0.33/2;
  public static final double poseThetaOffCenterA = Math.atan(poseYOffCenterA / poseXOffCenterA);
  public static final double poseThetaBetweenReefs = (Units.degreesToRadians(720) - poseThetaOffCenterA * 2 * 6) /(6);
  public static final double RadiusHexagon = Math.hypot(poseXOffCenterA, poseYOffCenterA);
  public static final Pose2d poseCenterReefBlue = new Pose2d(4.489323, 4.03352, new Rotation2d(0));
  




  


  public RobotState() {
    // //do all Hashmap stuff
    Pose2d scoringPoseABLUE = new Pose2d(new Translation2d(Math.cos( Math.PI - poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI - poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0));
    Pose2d[] As = {scoringPoseABLUE, getRedPose(scoringPoseABLUE)};
    PositionGetter.put(ScoringPosition.A, As);


    Pose2d scoringPoseBBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI + poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0));
    Pose2d[] Bs = {scoringPoseBBLUE, getRedPose(scoringPoseBBLUE)};
    PositionGetter.put(ScoringPosition.B, Bs);

    Pose2d scoringPoseCBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + poseThetaOffCenterA + poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + poseThetaOffCenterA + poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60));
    Pose2d[] Cs = {scoringPoseCBLUE, getRedPose(scoringPoseCBLUE)};
    PositionGetter.put(ScoringPosition.C, Cs);

    Pose2d scoringPoseDBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 3 * poseThetaOffCenterA  + poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 3 * poseThetaOffCenterA + poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60));
    Pose2d[] Ds = {scoringPoseDBLUE, getRedPose(scoringPoseDBLUE)};
    PositionGetter.put(ScoringPosition.D, Ds);

    
    Pose2d scoringPoseEBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 3 * poseThetaOffCenterA  + 2 *poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 3 * poseThetaOffCenterA + 2 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120));
    Pose2d[] Es = {scoringPoseEBLUE, getRedPose(scoringPoseEBLUE)};
    PositionGetter.put(ScoringPosition.E, Es);


    
    Pose2d scoringPoseFBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 5 * poseThetaOffCenterA  + 2 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 5 * poseThetaOffCenterA + 2 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120));
    Pose2d[] Fs = {scoringPoseFBLUE, getRedPose(scoringPoseFBLUE)};
    PositionGetter.put(ScoringPosition.F, Fs);

     
    Pose2d scoringPoseGBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 5 * poseThetaOffCenterA  + 3 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 5 * poseThetaOffCenterA + 3 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180));
    Pose2d[] Gs = {scoringPoseGBLUE, getRedPose(scoringPoseGBLUE)};
    PositionGetter.put(ScoringPosition.G, Gs);

    Pose2d scoringPoseHBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 7 * poseThetaOffCenterA  + 3 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 7 * poseThetaOffCenterA + 3 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180));
    Pose2d[] Hs = {scoringPoseHBLUE, getRedPose(scoringPoseHBLUE)};
    PositionGetter.put(ScoringPosition.H, Hs);

    Pose2d scoringPoseIBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 7 * poseThetaOffCenterA  + 4 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 7 * poseThetaOffCenterA + 4 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240));
    Pose2d[] Is = {scoringPoseIBLUE, getRedPose(scoringPoseIBLUE)};
    PositionGetter.put(ScoringPosition.I, Is);

    Pose2d scoringPoseJBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 9 * poseThetaOffCenterA  + 4 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 9 * poseThetaOffCenterA + 4 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240));
    Pose2d[] Js = {scoringPoseJBLUE, getRedPose(scoringPoseJBLUE)};
    PositionGetter.put(ScoringPosition.J, Js);

    Pose2d scoringPoseKBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 9 * poseThetaOffCenterA  + 5 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 9 * poseThetaOffCenterA + 5 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300));
    Pose2d[] Ks = {scoringPoseKBLUE, getRedPose(scoringPoseKBLUE)};
    PositionGetter.put(ScoringPosition.K, Ks);

    Pose2d scoringPoseLBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 11 * poseThetaOffCenterA  + 5 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 11 * poseThetaOffCenterA + 5 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300));
    Pose2d[] Ls = {scoringPoseLBLUE, getRedPose(scoringPoseLBLUE)};
    PositionGetter.put(ScoringPosition.L, Ls);




    






    





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


  public Pose2d getScoringPose() {
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      return PositionGetter.get(currentScoringCommand)[1];


    }

    else {
      return PositionGetter.get(currentScoringCommand)[0];
    }
    
  }

  public Pose2d[] getScoringPose_offset() {
    return PositionGetter_offset.get(currentScoringCommand);
  }

  // public boolean isRobotFarEnough(Pose2d currentPose) {
  //   Pose2d scoringPose = DriverStation.getAlliance().get() == Alliance.Blue ? getScoringPose()[0] : getScoringPose()[1];

  //   return currentPose.getTranslation().minus(scoringPose.getTranslation()).getNorm() > 1;

  // }


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
