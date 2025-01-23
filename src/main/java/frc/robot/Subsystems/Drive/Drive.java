package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = 200;


   

      

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
   private final Field2d m_field = new Field2d();
   private ReentrantLock poseLock = new ReentrantLock();
// Do this in either robot or subsystem init

  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  @AutoLogOutput
  double prevaccelX = 0;

  @AutoLogOutput
  double prevaccelY = 0;

  @AutoLogOutput
  double prevTime = 0;
 

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  //private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = SwerveConstants.swerveKinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator SwervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation(), lastModulePositions, new Pose2d(15.9, 5.78, new Rotation2d()), VecBuilder.fill(0.001, 0.001, Radians.convertFrom(5, Degrees)), VecBuilder.fill(0.1, 0.1, 999999) );
  
  
  
  
  
  
    private int numTimes = 0;
  
  
    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
      //this.initTime = Timer.getFPGATimestamp();
      this.gyroIO = gyroIO;
      modules[0] = new Module(flModuleIO,0, SwerveConstants.Mod0.constants);
      modules[1] = new Module(frModuleIO, 1, SwerveConstants.Mod1.constants);
      modules[2] = new Module(blModuleIO, 2, SwerveConstants.Mod2.constants);
      modules[3] = new Module(brModuleIO, 3,SwerveConstants.Mod3.constants);
  
      // Usage reporting for swerve template
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
      SmartDashboard.putData("Field", m_field);
      // Start odometry thread
      PhoenixOdometryThread.getInstance().start();
  
      // Configure AutoBuilder for PathPlanner
      // AutoBuilder.configure(
      //     this::getPose,
      //     this::setPose,
      //     this::getChassisSpeeds,
      //     this::runVelocity,
      //     new PPHolonomicDriveController(
      //         new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
      //     PP_CONFIG,
      //     () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      //     this);
      // Pathfinding.setPathfinder(new LocalADStarAK());
      // PathPlannerLogging.setLogActivePathCallback(
      //     (activePath) -> {
      //       Logger.recordOutput(
      //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      //     });
      // PathPlannerLogging.setLogTargetPoseCallback(
      //     (targetPose) -> {
      //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
      //     });
  
      // Configure SysId
      // sysId =
      //     new SysIdRoutine(
      //         new SysIdRoutine.Config(
      //             null,
      //             null,
      //             null,
      //             (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
      //         new SysIdRoutine.Mechanism(
      //             (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("bop bop", numTimes);
      odometryLock.lock(); // Prevents odometry updates while reading data
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
      odometryLock.unlock();

  
      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        for (var module : modules) {
          module.stop();
        }
      }
  
      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }
  
      // Update odometry
      double[] sampleTimestamps =
          modules[0].getOdometryTimestamps(); // All signals are sampled together
      int sampleCount = sampleTimestamps.length;
  
      
      for (int i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        double vx = 0;
        double vy = 0;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }
  
        // Update gyro angle
        if (gyroInputs.connected) {
          // Use the real gyro angle
          rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
          // Use the angle delta from the kinematics and module deltas
          Twist2d twist = kinematics.toTwist2d(moduleDeltas);
         
          
          
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
  
        // Apply update
        // int sum = 0;
  
  
        // for (SwerveModulePosition pos: moduleDeltas) {
        //   sum += pos.distanceMeters;
        // }
  
        // double avg_movement = sum/4;
        // Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        // vx = twist.dx * Math.cos(rawGyroRotation.getRadians()) + twist.dy * Math.sin(rawGyroRotation.getRadians());
        // vy = -twist.dx * Math.sin(rawGyroRotation.getRadians()) + twist.dy * Math.cos(rawGyroRotation.getRadians());
  
  
  
        // double stdx = avg_movement * vx * Constants.SwerveConstants.odometryConstant;
        // double stdy = avg_movement * vy * Constants.SwerveConstants.odometryConstant;
  
        // double jerkX = (gyroInputs.odometryaccelXpositions[i] - prevaccelX) / (gyroInputs.odometryYawTimestamps[i] - prevTime);
        // double jerkY = (gyroInputs.odometryaccelXpositions[i] - prevaccelY) / (gyroInputs.odometryYawTimestamps[i] - prevTime);
  
        // prevTime = gyroInputs.odometryYawTimestamps[i];
  
        // prevaccelX = gyroInputs.odometryaccelXpositions[i];
        // prevaccelY = gyroInputs.odometryaccelXpositions[i];
  
  
        // if (Math.sqrt(jerkX * jerkX + jerkY * jerkY) >  SwerveConstants.maxJerk) {
        //   stdx += Math.sqrt(jerkX * jerkX + jerkY * jerkY) * Constants.SwerveConstants.collisionMultiplier;
        //   stdy += Math.sqrt(jerkX * jerkX + jerkY * jerkY) * Constants.SwerveConstants.collisionMultiplier;
          
  
        // }
        poseLock.lock();
        SwervePoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        poseLock.unlock();
      }
        
        
        m_field.setRobotPose(SwervePoseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("x-val", SwervePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("y-val", SwervePoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("gyro", SwervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
      
  
      // Update gyro alert
      gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
     // m_field.setRobotPose(getPose());
  }
      
     // gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
     
  
    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
      // Calculate module setpoints
      ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4);
  
      // Log unoptimized setpoints and setpoint speeds
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
  
      // Send setpoints to modules
      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }
  
      // Log optimized setpoints (runSetpoint mutates each state)
      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }
  
    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
      for (int i = 0; i < 4; i++) {
        modules[i].runCharacterization(output);
      }
    }
  
    /** Stops the drive. */
    public void stop() {
      runVelocity(new ChassisSpeeds());
    }
  
    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
  //   public void stopWithX() {
  //     Rotation2d[] headings = new Rotation2d[4];
  //     for (int i = 0; i < 4; i++) {
  //       headings[i] = getModuleTranslations()[i].getAngle();
  //     }
  //     kinematics.resetHeadings(headings);
  //     stop();
  //   }
  
    /** Returns a command to run a quasistatic test in the specified direction. */
  //   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return run(() -> runCharacterization(0.0))
  //         .withTimeout(1.0)
  //         .andThen(sysId.quasistatic(direction));
  //   }
  
    /** Returns a command to run a dynamic test in the specified direction. */
  //   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  //   }
  
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
      }
      return states;
    }
  
  
    public Pose2d getPose() {
      return SwervePoseEstimator.getEstimatedPosition();
    }
  
  
    // public void resetGyro() {
    //   gyroIO.resetGyro();
    // }
  
    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] states = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
      }
      return states;
    }
  
    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }
  
    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
      double[] values = new double[4];
      for (int i = 0; i < 4; i++) {
        values[i] = modules[i].getWheelRadiusCharacterizationPosition();
      }
      return values;
    }
  
  
    public void addvision(Pose2d pose, double timestamp, double[] stds) {
      //SmartDashboard.putNumber("stds 0", stds[0]);
      //SmartDashboard.putNumber("stds 1", stds[1]);
     // SmartDashboard.putNumber("stds 2", stds[2]);
    //  numTimes++;
   // SwervePoseEstimator.addVisionMeasurement()
   
   //poseLock.lock();
  // Translation2d before = getPose().getTranslation();
   //SmartDashboard.putNumber("timestamp", timestamp);
   //SmartDashboard.putNumber("fpga", Timer.getFPGATimestamp());
    SwervePoseEstimator.addVisionMeasurement(pose, timestamp, VecBuilder.fill(stds[0], stds[1], stds[2]));
    //Translation2d after = getPose().getTranslation();
    //SmartDashboard.putNumber("translation diff", after.minus(before).getNorm());
    //poseLock.unlock();
   // SmartDashboard.putBoolean("aded", true);
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  // @AutoLogOutput(key = "Odometry/Robot")
  // public Pose2d getPose() {
  //   return poseEstimator.getEstimatedPosition();
  // }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return gyroInputs.yawPosition;
  }

  /** Resets the current odometry pose. */
  // public void setPose(Pose2d pose) {
  //   poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  // }

  /** Adds a new timestamped vision measurement. */
  // public void addVisionMeasurement(
  //     Pose2d visionRobotPoseMeters,
  //     double timestampSeconds,
  //     Matrix<N3, N1> visionMeasurementStdDevs) {
  //   poseEstimator.addVisionMeasurement(
  //       visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  // }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return 4;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / SwerveConstants.DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  
}