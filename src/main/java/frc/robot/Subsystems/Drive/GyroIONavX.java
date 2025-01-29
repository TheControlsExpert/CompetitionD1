package frc.robot.Subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, 200);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> jerkXQueue;
  private final Queue<Double> jerkYQueue;
  private boolean resetHasntHappened = false;

  public GyroIONavX() {
    
    
    SmartDashboard.putNumber("reset pos", navX.getYaw());
    
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getAngle);
    jerkXQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getWorldLinearAccelX);
    jerkYQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getWorldLinearAccelY);

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
      navX.setAngleAdjustment(-54);
      
      resetHasntHappened = true;
    }

    SmartDashboard.putNumber("gyro actual", navX.getYaw());
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw()-54);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);

     inputs.odometryaccelXpositions = jerkXQueue.stream().mapToDouble((Double value) -> value).toArray();
     inputs.odometryaccelYpositions = jerkYQueue.stream().mapToDouble((Double value) -> value).toArray();
        
    LimelightHelpers.SetRobotOrientation("limelight-threegf", -navX.getYaw() + 54 + 180, 0, 0, 0, 0, 0);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}