package frc.robot;

import java.util.HashMap;

import org.jgrapht.alg.color.SmallestDegreeLastColoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;
import frc.robot.Subsystems.Superstructure.Superstructure;

public class RobotState {

  public static RobotState instance;
    
  public static Superstructure.SuperstructureState currentScoringLevel = Superstructure.SuperstructureState.L4_STOWED;
  public static ScoringPosition currentScoringCommand = ScoringPosition.A;
  public double fieldMiddleX = 8.774176;
  public double fieldMiddleY = 4.0259;

  public static HashMap< ScoringPosition, Pose2d[]> PositionGetter = new HashMap<>();
  public static HashMap< Pose2d, AlgaeLevel> AlgaeBlue = new HashMap<>();
  public static HashMap< Pose2d, AlgaeLevel> AlgaeRed = new HashMap<>();
  public static HashMap< ScoringPosition, Integer[]> aprilTagIDGetter = new HashMap<>();
  public static HashMap<ScoringPosition, DirectionREEF> DirectionGetter = new HashMap<>();


  public static final double poseXOffCenterA = 1.35;
  
  //public static final double poseYOffCenterA = 0.33/2;
  public static final double poseYOffCenterA = 0;
  public static final double poseThetaOffCenterA = Math.atan(poseYOffCenterA / poseXOffCenterA);
  public static final double poseThetaBetweenReefs = (Units.degreesToRadians(360) - poseThetaOffCenterA * 2 * 6) /(6);
  public static final double RadiusHexagon = poseXOffCenterA;
  public static final Pose2d poseCenterReefBlue = new Pose2d(4.489323, 4.03352, new Rotation2d(0));

  




  


  public RobotState() {
    // //do all Hashmap stuff

    DirectionGetter.put(ScoringPosition.A, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.B, DirectionREEF.RIGHT);


    DirectionGetter.put(ScoringPosition.C, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.D, DirectionREEF.RIGHT);

    DirectionGetter.put(ScoringPosition.E, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.F, DirectionREEF.RIGHT);

    DirectionGetter.put(ScoringPosition.G, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.H, DirectionREEF.RIGHT);

    DirectionGetter.put(ScoringPosition.I, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.J, DirectionREEF.RIGHT);

    DirectionGetter.put(ScoringPosition.K, DirectionREEF.LEFT);
    DirectionGetter.put(ScoringPosition.L, DirectionREEF.RIGHT);




    Pose2d scoringPoseABLUE = new Pose2d(new Translation2d(Math.cos(Math.PI) * RadiusHexagon, Math.sin(Math.PI) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d());
    //Pose2d scoringPoseABLUE = new Pose2d(new Translation2d(Math.cos( Math.PI - poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI - poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0).minus(Rotation2d.fromDegrees(2)));
    Pose2d[] As = {scoringPoseABLUE, getRedPose(scoringPoseABLUE)};
    PositionGetter.put(ScoringPosition.A, As);

    Pose2d scoringPoseBBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI) * RadiusHexagon, Math.sin(Math.PI) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d());

   // Pose2d scoringPoseBBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + poseThetaOffCenterA) * RadiusHexagon, Math.sin(Math.PI + poseThetaOffCenterA) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), new Rotation2d(0).plus(Rotation2d.fromDegrees(10)));
    Pose2d[] Bs = {scoringPoseBBLUE, getRedPose(scoringPoseBBLUE)};
    PositionGetter.put(ScoringPosition.B, Bs);

    Pose2d scoringPoseCBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(240)) * RadiusHexagon, Math.sin(Units.degreesToRadians(240)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60));
    Pose2d[] Cs = {scoringPoseCBLUE, getRedPose(scoringPoseCBLUE)};
    PositionGetter.put(ScoringPosition.C, Cs);

    Pose2d scoringPoseDBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(240)) * RadiusHexagon, Math.sin(Units.degreesToRadians(240)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60));

   // Pose2d scoringPoseDBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 3 * poseThetaOffCenterA  + poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 3 * poseThetaOffCenterA + poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60).plus(Rotation2d.fromDegrees(2)));
    Pose2d[] Ds = {scoringPoseDBLUE, getRedPose(scoringPoseDBLUE)};
    PositionGetter.put(ScoringPosition.D, Ds);

    Pose2d scoringPoseEBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(300)) * RadiusHexagon, Math.sin(Units.degreesToRadians(300)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120));

  //  Pose2d scoringPoseEBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 3 * poseThetaOffCenterA  + 2 *poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 3 * poseThetaOffCenterA + 2 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120).minus(Rotation2d.fromDegrees(2)));
    Pose2d[] Es = {scoringPoseEBLUE, getRedPose(scoringPoseEBLUE)};
    PositionGetter.put(ScoringPosition.E, Es);


    Pose2d scoringPoseFBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(300)) * RadiusHexagon, Math.sin(Units.degreesToRadians(300)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120));

    //Pose2d scoringPoseFBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 5 * poseThetaOffCenterA  + 2 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 5 * poseThetaOffCenterA + 2 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120).plus(Rotation2d.fromDegrees(2)));
    Pose2d[] Fs = {scoringPoseFBLUE, getRedPose(scoringPoseFBLUE)};
    PositionGetter.put(ScoringPosition.F, Fs);

     
    Pose2d scoringPoseGBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(0)) * RadiusHexagon, Math.sin(Units.degreesToRadians(0)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180));

   // Pose2d scoringPoseGBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 5 * poseThetaOffCenterA  + 3 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 5 * poseThetaOffCenterA + 3 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180).minus(Rotation2d.fromDegrees(2)));
    Pose2d[] Gs = {scoringPoseGBLUE, getRedPose(scoringPoseGBLUE)};
    PositionGetter.put(ScoringPosition.G, Gs);


    Pose2d scoringPoseHBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(0)) * RadiusHexagon, Math.sin(Units.degreesToRadians(0)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180));

   // Pose2d scoringPoseHBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 7 * poseThetaOffCenterA  + 3 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 7 * poseThetaOffCenterA + 3 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180).plus(Rotation2d.fromDegrees(2)));
    Pose2d[] Hs = {scoringPoseHBLUE, getRedPose(scoringPoseHBLUE)};
    PositionGetter.put(ScoringPosition.H, Hs);

    Pose2d scoringPoseIBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(60)) * RadiusHexagon, Math.sin(Units.degreesToRadians(60)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240));

    //Pose2d scoringPoseIBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 7 * poseThetaOffCenterA  + 4 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 7 * poseThetaOffCenterA + 4 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240).minus(Rotation2d.fromDegrees(2)));
    Pose2d[] Is = {scoringPoseIBLUE, getRedPose(scoringPoseIBLUE)};
    PositionGetter.put(ScoringPosition.I, Is);


    Pose2d scoringPoseJBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(60)) * RadiusHexagon, Math.sin(Units.degreesToRadians(60)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240));

    //Pose2d scoringPoseJBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 9 * poseThetaOffCenterA  + 4 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 9 * poseThetaOffCenterA + 4 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240).plus(Rotation2d.fromDegrees(2)));
    Pose2d[] Js = {scoringPoseJBLUE, getRedPose(scoringPoseJBLUE)};
    PositionGetter.put(ScoringPosition.J, Js);

   
    Pose2d scoringPoseKBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(120)) * RadiusHexagon, Math.sin(Units.degreesToRadians(120)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300));

   // Pose2d scoringPoseKBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 9 * poseThetaOffCenterA  + 5 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 9 * poseThetaOffCenterA + 5 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300).minus(Rotation2d.fromDegrees(2)));
    Pose2d[] Ks = {scoringPoseKBLUE, getRedPose(scoringPoseKBLUE)};
    PositionGetter.put(ScoringPosition.K, Ks);


    Pose2d scoringPoseLBLUE = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(120)) * RadiusHexagon, Math.sin(Units.degreesToRadians(120)) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300));

    //Pose2d scoringPoseLBLUE = new Pose2d(new Translation2d(Math.cos(Math.PI + 11 * poseThetaOffCenterA  + 5 * poseThetaBetweenReefs) * RadiusHexagon, Math.sin(Math.PI + 11 * poseThetaOffCenterA + 5 * poseThetaBetweenReefs) * RadiusHexagon).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300).plus(Rotation2d.fromDegrees(2)));
    Pose2d[] Ls = {scoringPoseLBLUE, getRedPose(scoringPoseLBLUE)};
    PositionGetter.put(ScoringPosition.L, Ls);



   
Pose2d AB_Blue = new Pose2d(new Translation2d(Math.cos(Math.PI) * 1, Math.sin(Math.PI) * 1).plus(poseCenterReefBlue.getTranslation()), new Rotation2d());
Pose2d CD_Blue = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(240)), Math.sin(Units.degreesToRadians(240))).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(60));
Pose2d EF_Blue = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(300)), Math.sin(Units.degreesToRadians(300))).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(120));
Pose2d GH_Blue = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(0)), Math.sin(Units.degreesToRadians(0))).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(180));
Pose2d IJ_Blue = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(60)), Math.sin(Units.degreesToRadians(60))).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(240));
Pose2d KL_Blue = new Pose2d(new Translation2d(Math.cos(Units.degreesToRadians(120)), Math.sin(Units.degreesToRadians(120))).plus(poseCenterReefBlue.getTranslation()), Rotation2d.fromDegrees(300));
    AlgaeBlue.put(AB_Blue, AlgaeLevel.L3);
    AlgaeBlue.put(CD_Blue, AlgaeLevel.L2);
    AlgaeBlue.put(EF_Blue, AlgaeLevel.L3);
    AlgaeBlue.put(GH_Blue, AlgaeLevel.L2);
    AlgaeBlue.put(IJ_Blue, AlgaeLevel.L3);
    AlgaeBlue.put(KL_Blue, AlgaeLevel.L2);


    Pose2d AB_red = getRedPose(AB_Blue);
    Pose2d CD_red = getRedPose(CD_Blue);
    Pose2d EF_red = getRedPose(EF_Blue);
    Pose2d GH_red = getRedPose(GH_Blue);
    Pose2d IJ_red = getRedPose(IJ_Blue);
    Pose2d KL_red = getRedPose(KL_Blue);

    AlgaeRed.put(AB_red, AlgaeLevel.L3);
    AlgaeRed.put(CD_red, AlgaeLevel.L2);
    AlgaeRed.put(EF_red, AlgaeLevel.L3);
    AlgaeRed.put(GH_red, AlgaeLevel.L2);
    AlgaeRed.put(IJ_red, AlgaeLevel.L3);
    AlgaeRed.put(KL_red, AlgaeLevel.L2);


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


//   public AlgaeState getAlgaePose() {
//     //enter stuff here
//   }

  


  public Pose2d getScoringPose() {
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return PositionGetter.get(Robot.CurrnetScoringPosition)[1];


    }

    else {
      return PositionGetter.get(Robot.CurrnetScoringPosition)[0];
    }

    
  }

  public AlgaeLevel getAlgaeLevel(Pose2d currentPose) {
   

        double minDistance = 100000000;
        Pose2d minPose = new Pose2d();
        boolean isBlue = true;

        for(Pose2d pose: AlgaeBlue.keySet().toArray(new Pose2d[6])) {
            if (currentPose.minus(pose).getTranslation().getNorm() < minDistance) {
                minDistance = currentPose.minus(pose).getTranslation().getNorm();
                minPose = pose;
                isBlue = true;
            }

        }


        for(Pose2d pose: AlgaeRed.keySet().toArray(new Pose2d[6])) {
          if (currentPose.minus(pose).getTranslation().getNorm() < minDistance) {
              minDistance = currentPose.minus(pose).getTranslation().getNorm();
              minPose = pose;
              isBlue = false;
          }
        }
    
            //so we dont get an error from our hashmap
            if (isBlue) {
            return AlgaeBlue.get(minPose);     
            }

            else {
              return AlgaeRed.get(minPose);
            }
   
  }


  public Pose2d getClosestAlgaeReef(Pose2d currentPose) {

    
    double minDistance = 100000000;
    Pose2d minPose = new Pose2d();
  
    for(Pose2d pose: AlgaeBlue.keySet().toArray(new Pose2d[6])) {
        if (currentPose.minus(pose).getTranslation().getNorm() < minDistance) {
            minDistance = currentPose.minus(pose).getTranslation().getNorm();
            minPose = pose;
           
        }

    }


    for(Pose2d pose: AlgaeRed.keySet().toArray(new Pose2d[6])) {
      if (currentPose.minus(pose).getTranslation().getNorm() < minDistance) {
          minDistance = currentPose.minus(pose).getTranslation().getNorm();
          minPose = pose;
          
      }
    }


    return minPose;

  }




  


  // public boolean isRobotFarEnough(Pose2d currentPose) {
  //   Pose2d scoringPose = DriverStation.getAlliance().get() == Alliance.Blue ? getScoringPose()[0] : getScoringPose()[1];

  //   return currentPose.getTranslation().minus(scoringPose.getTranslation()).getNorm() > 1;

  // }


  public Pose2d getIntakingPose(Pose2d currentPose) {
   

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
      double distanceR = Math.abs(currentPose.getTranslation().minus(FieldConstants.RightSource_BLUE.getTranslation()).getNorm());
      double distanceL = Math.abs(currentPose.getTranslation().minus(FieldConstants.LeftSource_BLUE.getTranslation()).getNorm());

      if (distanceL >= distanceR) {
        //since we are closer to the right source, go there
        return FieldConstants.RightSource_BLUE;
      }

      else {
        return FieldConstants.LeftSource_BLUE;
      }

    }

    
    else {
      double distanceR = Math.abs(getRedPose(FieldConstants.RightSource_BLUE).getTranslation().minus(currentPose.getTranslation()).getNorm());
      double distanceL = Math.abs(getRedPose(FieldConstants.LeftSource_BLUE).getTranslation().minus(currentPose.getTranslation()).getNorm());

      if (distanceL >= distanceR) {
        //since we are closer to the right source, go there
        SmartDashboard.putNumber("intaking pose x", FieldConstants.RightSource_BLUE.getX());
        SmartDashboard.putNumber("intaking pose y", FieldConstants.RightSource_BLUE.getY());
        return getRedPose(FieldConstants.RightSource_BLUE);
      }

      else {
        SmartDashboard.putNumber("intaking pose x", FieldConstants.LeftSource_BLUE.getX());
        SmartDashboard.putNumber("intaking pose y", FieldConstants.LeftSource_BLUE.getY());
        return getRedPose(FieldConstants.LeftSource_BLUE);
      }

    }
  }


  public static enum AlgaeLevel{
    L2,
    L3,
    //closeset to the driver is L3
}

  public static enum DirectionREEF {
    LEFT,
    RIGHT
  }

  


  public DirectionREEF getScoringDirection() {
    return DirectionGetter.get(Robot.CurrnetScoringPosition);
  }

    
}
