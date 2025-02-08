package frc.robot.Subsystems.Vision;

import java.lang.reflect.Field;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
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
        
        
        public VisionSubsystem(VisionIO io, Drive drive) {
                this.io = io;
                this.drive = drive;
               //SmartDashboard.putData("field", field);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (inputs.isNew_LL3GF & !inputs.MT2pose_LL3GF.equals(new Pose2d()) & inputs.visionSTDs_LL3GF[0] != 0 && inputs.visionSTDs_LL3GF[1] !=0) {
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
        Vector<N2> stds = VecBuilder.fill(std[0], std[1]);
        
        
        drive.addVision(pose, timestamp, std);
       //field.setRobotPose(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(-drive.getRotationLL())));

        }


    }
    
}
