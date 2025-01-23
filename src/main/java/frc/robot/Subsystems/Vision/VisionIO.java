package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {


   @AutoLog

     public class VisionIOInputs {
        public double time_LL3GF = 0;
        public Pose2d MT2pose_LL3GF = new Pose2d();
        public double fps_LL3GF = 0;
        public double tagCount_LL3GF = 0;    
        public double avgArea_LL3GF = 0;
        public double avgDistance_LL3GF = 0;
        public boolean isNew_LL3GF = false;
        public boolean isConnected_LL3GF = false;
        public double[] visionSTDs_LL3GF = new double[3];

        public double latency_LL3GS = 0;
        public Pose2d MT2pose_LL3GS = new Pose2d();
        public double fps_LL3GS = 0;
        public int tagCount_LL3GS = 0;
        public double maxArea_LL3GS = 0;
        public double avgArea_LL3GS = 0;
        public double avgDistance_LL3GS = 0;
        public boolean isNew_LL3GS = false;
        public boolean isConnected_LL3GS = false;

        public double latency_LL3 = 0;
        public Pose2d MT2pose_LL3 = new Pose2d();
        public double fps_LL3 = 0;
        public int tagCount_LL3 = 0;
        public double maxArea_LL3 = 0;
        public double avgArea_LL3 = 0;
        public double avgDistance_LL3 = 0;
        public boolean isNew_LL3 = false;
        public boolean isConnected_LL3 = false;
     
    }

    default void updateInputs(VisionIOInputs inputs) {}


    default void setLED(int n) {}

    //0 = april tag vision
    //1 = object detection
    default void changePipeline(int n) {}
 

    
 
}
