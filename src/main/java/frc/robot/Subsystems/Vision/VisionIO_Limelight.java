package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIO_Limelight implements VisionIO {

    Pose2d oldpose = new Pose2d();
    DoubleArraySubscriber Limelight_3GF = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber Limelight_3GF_stds = NetworkTableInstance.getDefault().getTable("limelight-threegf").getDoubleArrayTopic("stddevs").subscribe(new double[12], PubSubOption.keepDuplicates(true));
    

 

    public void updateInputs(VisionIOInputs inputs) {
        //DoubleArrayTopic Limelight_3GF_pose = NetworkTableInstance.getDefault().getTable("limelight-3gf").getDoubleArrayTopic("botpose_orb_wpiblue");
        
        
        inputs.isConnected_LL3GF = true;
        TimestampedDoubleArray shit = Limelight_3GF.getAtomic();
        TimestampedDoubleArray shit_std = Limelight_3GF_stds.getAtomic();
        SmartDashboard.putNumber("shit.server", shit.serverTime/1000000.0);
        SmartDashboard.putNumber("value 6 - latency", shit.value[6]);


        double timestamp = shit.serverTime/1000000.0 - shit.value[6]/1000.0;
        double[] data_general = NetworkTableInstance.getDefault().getTable("limelight-threegf").getEntry("hw").getDoubleArray(new double[5]);
        inputs.MT2pose_LL3GF = new Pose2d(new Translation2d(shit.value[0], shit.value[1]), Rotation2d.fromDegrees(shit.value[5]));
        inputs.avgArea_LL3GF = shit.value[10];
        inputs.avgDistance_LL3GF = shit.value[9];
        inputs.fps_LL3GF = data_general[0];
        inputs.isNew_LL3GF = !inputs.MT2pose_LL3GF.getTranslation().equals(oldpose.getTranslation());
        oldpose = inputs.MT2pose_LL3GF;
        inputs.time_LL3GF = timestamp;
        inputs.tagCount_LL3GF = shit.value[7];
        double[] std = {shit_std.value[6], shit_std.value[7], 999999999};
        SmartDashboard.putNumber("std vision x", std[0]);
        SmartDashboard.putNumber("std vision Y", std[1]);

        inputs.visionSTDs_LL3GF = std;




    }


    
}
