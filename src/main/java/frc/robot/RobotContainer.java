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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import frc.robot.Commands.AutoDriveCommand;
import frc.robot.Commands.AutoScoreAimCommand;
import frc.robot.Commands.AutoSourcingCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.FeedforwardCharacterization;
import frc.robot.Commands.IntakingCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIONavX;
import frc.robot.Subsystems.Drive.ModuleIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Superstructure.ElevatorIOKrakens;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.WristIOKrakens;
import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;
import frc.robot.Subsystems.Vision.VisionIO_Limelight;
import frc.robot.Subsystems.Vision.VisionSubsystem;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake; 

 private final PathConstraints constraints;
  private Command pathfindingCommand;




       
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final AutoDriveCommand driver = new AutoDriveCommand(2, 0.03, 0, 1);
  Superstructure superstructure;
  
    private GyroIONavX gyro;
    
  
    // Dashboard inputs
    // final LoggedDashboardChooser<Command> autoChooser;
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
     this.intake = new Intake();
      this.gyro = new GyroIONavX();
    
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                gyro,
                new ModuleIOTalonFX(Mod0.constants, 0),
                new ModuleIOTalonFX(Mod1.constants, 1),
                new ModuleIOTalonFX(Mod2.constants, 2),
                new ModuleIOTalonFX(Mod3.constants, 3));

        superstructure = new Superstructure(new WristIOKrakens(), new ElevatorIOKrakens(), drive);        
       

      //  VisionSubsystem vision = new VisionSubsystem(new VisionIO_Limelight(), drive);

         constraints = new PathConstraints(
          2.0, 4.0,
          Units.degreesToRadians(400), Units.degreesToRadians(720));
  
  // Since AutoBuilder is configured, we can use it to build pathfinding commands
   pathfindingCommand = AutoBuilder.pathfindToPose(
          new Pose2d(16.30, 6.89, Rotation2d.fromDegrees(58)),
          constraints,
          0.0 // Goal end velocity in meters/sec
        
  );
  
    // Set up SysId routines
    //autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DriveCommand(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            drive));


//     controller.a().whileTrue(Commands.run(() -> 
    
// drive.runVelocity(
//     ChassisSpeeds.fromFieldRelativeSpeeds(
//       driver.getTargetSpeeds(drive.getEstimatedPosition(), new Pose2d(16.30, 6.89, Rotation2d.fromDegrees(58))),
//         isFlipped
//             ? drive.getRotation().plus(new Rotation2d(Math.PI))
//             : drive.getRotation())), drive));

//controller.x().whileTrue(pathfindingCommand);

controller.leftBumper().whileTrue(new AutoSourcingCommand(drive, superstructure, intake, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
controller.leftTrigger().whileTrue(Commands.either(new AutoScoreAimCommand(drive, superstructure, controller, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()), 
                                  Commands.runEnd(() -> {superstructure.setIntakeManual(0.2);}, () -> {superstructure.setIntakeManual(0);}, superstructure), 
                                  () -> (superstructure.DesiredManualMode.equals(ManualMode.MANUAL))));


controller.x().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setIntakeManual(-0.2);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setIntakeManual(0);}, superstructure));

//MANUAL MODES

controller.povUp().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setElevatorManual(0.2);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setElevatorManual(0);}, superstructure));
controller.povDown().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setElevatorManual(-0.1);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setElevatorManual(0);}, superstructure));
controller.povRight().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setWristManual(0.3);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setWristManual(0);}, superstructure));
controller.povLeft().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setWristManual(-0.3);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setWristManual(0);}, superstructure));

controller.y().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
controller.a().and(() -> superstructure.DesiredManualMode.equals(ManualMode.MANUAL)).whileTrue(new InstantCommand(() -> {superstructure.setPivotManual(-0.1 * 12);}, superstructure)).onFalse(new InstantCommand(() -> {superstructure.setPivotManual(0);}, superstructure));
    

controller.rightBumper().onTrue(Commands.runOnce(() -> { boolean whichSwitch = superstructure.getManualMode().equals(ManualMode.AUTOMATIC); if (whichSwitch) {superstructure.setDesiredManualMode(ManualMode.MANUAL);} else {superstructure.setDesiredManualMode(ManualMode.AUTOMATIC);}}, superstructure));
  

// controller.y().whileTrue(


//       new SelectCommand<>(
//           // Maps selector values to commands
//           Map.ofEntries(
//               Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
//               Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
//               Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!"))),
//           this::selectPathCommand));




  


    
    
    
   
   // controller.y().onTrue(Commands.runOnce(() -> gyro.resetGyro()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return FeedforwardCharacterization.feedforwardCommand(drive);
}




public enum ScoringLevel {
  L1,
  L2,
  L3,
  L4
}


public enum ScoringPosition {
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L
}


// public enum ScoringCommand {
//   OFFSET_STRAIGHT,
//   CURVING
// }

// public static Pose2d getScoringPose_w_offset() {
//   return PositionGetter.get()


// }


// public static Pose2d getScoringPose() {

// }


// public ScoringCommand selectPathCommand() {

//   //check if we are in zone for collision

//   double[] positions = PositionGetter_offset.get(currentScoringCommand);
//   double[] xyposition_goal = new double[2];
//   double m;
//   double b;

//   if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
//     xyposition_goal[0] = positions[0];
//     xyposition_goal[1] = positions[1];

//   }

//   else {
//     xyposition_goal[0] = positions[2];
//     xyposition_goal[1] = positions[3];
//   }

//   m = (drive.getEstimatedPosition().getY() - xyposition_goal[1]) / (drive.getEstimatedPosition().getX() - xyposition_goal[0]);
//   b = xyposition_goal[1] + m * xyposition_goal[0];












// }
}