package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase{
   //Field2d field = new Field2d();


    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
        private Drive drive;
   
        
    double lastUsedTimestamp = -1000;
    private double minTranslation = 10000.0;   
    public double reeftransformX = 0;
    public double reeftransformY = 0; 
            
            
            public VisionSubsystem(VisionIO io, Drive drive) {
                    this.io = io;
                    this.drive = drive;
                   //SmartDashboard.putData("field", field);
    
        }
    
        @Override
        public void periodic() {
            io.updateInputs(inputs);

            reeftransformX = inputs.xOffset;
            reeftransformY = inputs.yOffset;
    
    
           
    
            // LL4, then LL3GF, then LL3GS
    
            ArrayList<Pose2d> posesLeft = new ArrayList<>();
            ArrayList<double[]> stdsLeft = new ArrayList<>();
            ArrayList<Double> timestampsLeft = new ArrayList<>();
    
            if (inputs.isNew_LL4) {
                SmartDashboard.putBoolean("ll4 is new", true);
            posesLeft.add(inputs.MT2pose_LL4);
            timestampsLeft.add(inputs.time_LL4);
            SmartDashboard.putNumber("time ll4", inputs.time_LL4);
            double[] stds = {inputs.avgDistance_LL4 * 0.03 + 0.02, inputs.avgDistance_LL4 * 0.03 + 0.02};
            stdsLeft.add(stds);
            }
    
            else {
                SmartDashboard.putBoolean("ll4 is new", false);
    
            }
    
            if (inputs.isNew_LL3GF && inputs.avgDistance_LL3GF > 0.000001 && inputs.avgDistance_LL3GF < 5) {
        //   SmartDashboard.putBoolean("is new ll3gf", inputs.isNew_LL3GF);
        //   posesLeft.add(inputs.MT2pose_LL3GF);
        //   timestampsLeft.add(inputs.time_LL3GF);
        //   double[] stds = {inputs.avgDistance_LL3GF * 0.06 + 0.02, inputs.avgDistance_LL3GF * 0.06 + 0.02};
        //   stdsLeft.add(stds);
          
            }
    
            if (inputs.isNew_LL3GS) {
              //  posesLeft.add(inputs.MT2pose_LL3GS);
              //  timestampsLeft.add(inputs.time_LL3GS);
             //   stdsLeft.add(inputs.visionSTDs_LL3GS);
              //  stdsLeft.add(times(10, inputs.visionSTDs_LL3GS));
            }
    
            if (!posesLeft.isEmpty()) {
                SmartDashboard.putBoolean("nto all poses empty?", true);
    
            Pose2d[] poses = new Pose2d[posesLeft.size()];
            double[] times = new double[posesLeft.size()];
            double[][] stds = new double[posesLeft.size()][2];
    
    
            //sorting by timestamp
            Pose2d currentMinPose = new Pose2d();
            double currentMinTimestamp = 999999;
            double[] currentMinSTDS = {0.0, 0.0};
            int currentMinIndex = 0;
    
            for (int i = 0; i<poses.length; i++) {
                for (int j = 0; j < poses.length - i; j++) {
                    if (currentMinTimestamp > timestampsLeft.get(j) ) {     
                        currentMinTimestamp = timestampsLeft.get(j);
                        currentMinPose = posesLeft.get(j);
                        currentMinSTDS = stdsLeft.get(j);
                        currentMinIndex = j;
    
                    }
                }
    
                poses[i] = currentMinPose;
                times[i] = currentMinTimestamp;
                stds[i] = currentMinSTDS;
                
                posesLeft.remove(currentMinIndex);
                timestampsLeft.remove(currentMinIndex);
                stdsLeft.remove(currentMinIndex);
                currentMinTimestamp = 99999;
            }
    
    
            for (int i = 0; i < poses.length; i++) {
                SmartDashboard.putNumberArray(" timestamp double[]", times);
                SmartDashboard.putNumber("difference in times", times[i] - lastUsedTimestamp);
                SmartDashboard.putBoolean("is trying to add", true);
                SmartDashboard.putBoolean("two", !poses[i].equals(new Pose2d()));
                SmartDashboard.putBoolean("three",  (poses[i].minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 1 || DriverStation.isDisabled()));
                if (times[i] > lastUsedTimestamp && poses[i].getTranslation().getNorm() > 0.1 && (poses[i].minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 2 || DriverStation.isDisabled())) {
                    lastUsedTimestamp = times[i];
                    SmartDashboard.putBoolean("is adding vision measurement to drive", true);
                    if (drive.getGyroSpeed() < 360) {

                    
                    addVisionMeasurement(poses[i], times[i], stds[i]);
                    }
                }
            }
    
        }
    
    
    
            
    
         
          
        }
    
    
        public double[] times(double multiplier, double[] list) {
            for (int i = 0; i < list.length; i++) {
                list[i] = list[i] * multiplier;
            }
    
            return list;
    
        }
    
    
        public void addVisionMeasurement(Pose2d pose, double timestamp, double[] std) {
            SmartDashboard.putBoolean("adding vision", true);
    
           //Rotation2d bob = pose.getRotation().plus(Rotation2d.fromDegrees(180));
           // Vector<N2> stds = VecBuilder.fill(std[0], std[1]);
           // SmartDashboard.putNumber("translation diff", pose.minus(drive.getEstimatedPosition()).getTranslation().getNorm());
    
            //if (pose.minus(drive.getEstimatedPosition()).getTranslation().getNorm() < 1) {
    
        
        drive.addVision(pose, timestamp, std);
       // }
       //field.setRobotPose(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(-drive.getRotationLL())));

        



    }
    }
    

