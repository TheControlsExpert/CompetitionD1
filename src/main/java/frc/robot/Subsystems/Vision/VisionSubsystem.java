package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase{
   //ield2d field = new Field2d();


    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
        private Drive drive;
        
        
        public VisionSubsystem(VisionIO io, Drive drive) {
                this.io = io;
                this.drive = drive;
               //artDashboard.putData("field", field);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (inputs.isNew_LL3GF) {
            SmartDashboard.putBoolean("adding measurement", true);
            addVisionMeasurement(inputs.MT2pose_LL3GF, inputs.time_LL3GF, inputs.visionSTDs_LL3GF);
        }

        else {
            SmartDashboard.putBoolean("adding measurement", false);

        }
      
    }


    public void addVisionMeasurement(Pose2d pose, double timestamp, double[] std) {
        if (pose.getTranslation().getNorm() >0.0001) {

        Rotation2d bob = pose.getRotation().plus(Rotation2d.fromDegrees(180));
        
        
        drive.addvision(new Pose2d(pose.getTranslation(), bob), timestamp, std);
       //ield.setRobotPose(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(-drive.getRotationLL())));

        }


    }
    
}
