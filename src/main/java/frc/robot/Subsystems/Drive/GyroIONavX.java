package frc.robot.Subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, 200);
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  double rotation_offset = 0;
  // private final Queue<Double> jerkXQueue;
  // private final Queue<Double> jerkYQueue;
  private boolean resetHasntHappened = false;

  public GyroIONavX() {
    
    
    SmartDashboard.putNumber("reset pos", navX.getYaw());
    
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
    //jerkXQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getWorldLinearAccelX);
    //jerkYQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getWorldLinearAccelY);

    // jerkpositionQueue = PhoenixOdometryThread.getInstance().registerSignal(() -> (Math.sqrt(
    //   navX.getRawAccelX() * navX.getRawAccelX() +
    //   navX.getRawAccelY() * navX.getRawAccelY() + 
    //   navX.getRawAccelZ() * navX.getRawAccelZ())));
    
    

    

  }



  public void resetGyro() {
    navX.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    if (!navX.isCalibrating() & !resetHasntHappened) {
      navX.zeroYaw();
      navX.reset();
      
      
      resetHasntHappened = true;
    }

    SmartDashboard.putNumber("gyro actual", navX.getYaw());
    inputs.connected = navX.isConnected();
    double addON = DriverStation.getAlliance().equals(DriverStation.Alliance.Red) ? 180 : 0;
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw()  + rotation_offset + addON);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);

     inputs.odometryaccelXpositions = accelerometer.getX();
     inputs.odometryaccelYpositions = accelerometer.getY();
    
        
    LimelightHelpers.SetRobotOrientation("limelight-threegf", -navX.getYaw() + rotation_offset  - 13 , 0 ,0 ,0 ,0 ,0 );

    LimelightHelpers.SetRobotOrientation("limelight-threegs", -navX.getYaw() + rotation_offset + 180 + 13 , 0 ,0 ,0 ,0 ,0 );
    LimelightHelpers.SetRobotOrientation("limelight-four", -navX.getYaw() + rotation_offset + 20 + addON, 0 ,0 ,0 ,0 ,0 );
    SmartDashboard.putBoolean("setting robot orientation", true);
    SmartDashboard.putNumber("reset pos",  -navX.getYaw() + rotation_offset + 180 - 13 + addON);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  public void resetGyro(Rotation2d rotation) {
    rotation_offset = rotation.getDegrees() - (-navX.getYaw());

  }
}