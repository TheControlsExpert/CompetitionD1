// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class SwerveConstants {

    // Gear Ratio
    public static final double driveReduction = 8.14;
    public static final double steerReduction = 150 / 7;

    /* Steer Motor PID Values */

    public static final double angleKP = 20;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKS = 0.2999;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.00; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.17; // TODO: This must be tuned to specific robot
    public static final double driveKV = 0.92;

    public static final Slot0Configs intrinsicsD =
        new Slot0Configs().withKP(driveKP).withKD(driveKD).withKV(driveKV).withKS(driveKS);
    public static final Slot0Configs instrinsicsS =
        new Slot0Configs().withKP(angleKP).withKD(angleKD).withKS(angleKS);

    public record SwerveModuleConstants(
        int driveMotorID,
        int angleMotorID,
        int canCoderID,
        double angleOffset,
        boolean invertEncoder,
        boolean invertDrive,
        boolean invertSteer) {}

    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.088;
      // 36.123046875 + 2.28515625)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 6;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = false;
      public static final boolean invertEncoder = false;

      public static final double angleOffset = -0.309082;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = 0.367;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 12;
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.17749;
      // -120.937
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    public static final double trackWidth = 0.63125;
    public static final double wheelBase = 0.58125;
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt(wheelBase * wheelBase / 4 + trackWidth * trackWidth / 4);
    public static final double WheelRadius = 0;
    public static final LinearVelocity MaxFreeSpeed = null;
    public static final double odometryConstant = 0;
    public static final double maxAccel = 0;
    public static final double maxJerk = 100;
    public static final double collisionMultiplier = 0;
    public static final double kAccel = 0;
    public static final double kMovement = 1;
  }
}
