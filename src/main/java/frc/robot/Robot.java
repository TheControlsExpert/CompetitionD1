// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Drive.GyroIONavX;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final RobotContainer m_robotContainer;
  private JoystickButton thirteen;
  private JoystickButton fourteen;
  private JoystickButton fifteen;
  private JoystickButton sixteen;
  public static levelscore CurrnetLevelPosition = levelscore.Level4;
  
    public Robot() {
     m_robotContainer = new RobotContainer();
     CurrnetLevelPosition = levelscore.Level4;
    GenericHID ButtonControllerLevels = new GenericHID(1);

       
   thirteen = new JoystickButton(ButtonControllerLevels, 1);
   fourteen = new JoystickButton(ButtonControllerLevels, 2);
   fifteen = new JoystickButton(ButtonControllerLevels, 3);
   sixteen = new JoystickButton(ButtonControllerLevels, 4);

    }
  
    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      //SmartDashboard.putNumber("accel", gyro.getWorldLinearAccelX());
      SmartDashboard.putString("CurrnetScoringPosition", CurrnetLevelPosition.toString());

     ButtonLevelscoring();
    }
  
    @Override
    public void disabledInit() {}

    
public static enum levelscore {
  Level1,
  Level2,
  Level3,
  Level4,

}

    public  void ButtonLevelscoring() {
      if (thirteen.getAsBoolean()) {  
        CurrnetLevelPosition = levelscore.Level1;
      }
      else if (fourteen.getAsBoolean()) {
        CurrnetLevelPosition = levelscore.Level2;
      }
      else if (fifteen.getAsBoolean()) {
        CurrnetLevelPosition = levelscore.Level3;
      }
      else if (sixteen.getAsBoolean()) {
        CurrnetLevelPosition = levelscore.Level4;
      }
    }
       


  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.superstructure.setDesiredState(SuperstructureState.HOME_UP);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


  
}
