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
    DoubleArraySubscriber Limelight_4 = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11], PubSubOption.keepDuplicates(true));
    DoubleArraySubscriber Limelight_4_stds = NetworkTableInstance.getDefault().getTable("limelight-four").getDoubleArrayTopic("stddevs").subscribe(new double[12], PubSubOption.keepDuplicates(true));
    

 

    public void updateInputs(VisionIOInputs inputs) {
        //DoubleArrayTopic Limelight_4_pose = NetworkTableInstance.getDefault().getTable("limelight-3gf").getDoubleArrayTopic("botpose_orb_wpiblue");
        
        
       // inputs.isConnected_LL4 = true;
        TimestampedDoubleArray shit = Limelight_4.getAtomic();
        TimestampedDoubleArray shit_std = Limelight_4_stds.getAtomic();
        SmartDashboard.putNumber("shit.server", shit.serverTime/1000000.0);
        SmartDashboard.putNumber("value 6 - latency", shit.value[6]);


        double timestamp = shit.serverTime/1000000.0 - shit.value[6]/1000.0;
        double[] data_general = NetworkTableInstance.getDefault().getTable("limelight-threegf").getEntry("hw").getDoubleArray(new double[5]);
        inputs.MT2pose_LL4 = new Pose2d(new Translation2d(shit.value[0], shit.value[1]), Rotation2d.fromDegrees(shit.value[5]));
        inputs.avgArea_LL4 = shit.value[10];
        inputs.avgDistance_LL4 = shit.value[9];
        inputs.fps_LL4 = data_general[0];
        inputs.isNew_LL4 = !inputs.MT2pose_LL4.getTranslation().equals(oldpose.getTranslation());
        oldpose = inputs.MT2pose_LL4;
        inputs.time_LL4 = timestamp;
        inputs.tagCount_LL4 = shit.value[7];
        double[] std = {shit_std.value[6], shit_std.value[7], 999999999};
        SmartDashboard.putNumber("std vision x", std[0]);
        SmartDashboard.putNumber("std vision Y", std[1]);

        inputs.visionSTDs_LL4 = std;




    }


    
}
