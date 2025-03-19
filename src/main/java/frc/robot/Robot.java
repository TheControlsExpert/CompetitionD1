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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot.levelscore;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Subsystems.Drive.GyroIONavX;
import frc.robot.Subsystems.Drive.PhoenixOdometryThread;
//import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final RobotContainer m_robotContainer;
  JoystickButton one;
  JoystickButton two;
  JoystickButton three;
  JoystickButton four;
  JoystickButton five;
  JoystickButton six;
  JoystickButton seven;
  JoystickButton eight;
  JoystickButton nine;
  JoystickButton ten;
  JoystickButton eleven;
  JoystickButton twelve;





  private JoystickButton thirteen;
  private JoystickButton fourteen;
  private JoystickButton fifteen;
  private JoystickButton sixteen;
  private JoystickButton seventeen;
  private JoystickButton eighteen;
  public static levelscore CurrnetLevelPosition = levelscore.Level3;
  public static ScoringPosition CurrnetScoringPosition = ScoringPosition.A;
  public static ReefMode reefMode = ReefMode.CORAL;
  
    public Robot() {
     m_robotContainer = new RobotContainer();
     CurrnetLevelPosition = levelscore.Level4;
    GenericHID ButtonControllerLevels = m_robotContainer.LevelsController;
    
    
    GenericHID ButtonControllerScoringposs = m_robotContainer.PositionsController;
    







     one = new JoystickButton(ButtonControllerScoringposs, 1);
   two = new JoystickButton(ButtonControllerScoringposs, 2);
   three = new JoystickButton(ButtonControllerScoringposs, 3);
   four = new JoystickButton(ButtonControllerScoringposs, 4);
   five = new JoystickButton(ButtonControllerScoringposs, 5);
   six = new JoystickButton(ButtonControllerScoringposs, 6);
   seven = new JoystickButton(ButtonControllerScoringposs, 7);
   eight = new JoystickButton(ButtonControllerScoringposs, 8);
   nine = new JoystickButton(ButtonControllerScoringposs, 9);
   ten = new JoystickButton(ButtonControllerScoringposs, 10);
   eleven = new JoystickButton(ButtonControllerScoringposs, 11);
   twelve = new JoystickButton(ButtonControllerScoringposs, 12);
       
   thirteen = new JoystickButton(ButtonControllerLevels, 1);
   fourteen = new JoystickButton(ButtonControllerLevels, 2);
   fifteen = new JoystickButton(ButtonControllerLevels, 3);
   sixteen = new JoystickButton(ButtonControllerLevels, 4);
   seventeen = new JoystickButton(ButtonControllerLevels, 5);
   eighteen = new JoystickButton(ButtonControllerLevels, 6);
   




    }
  
    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      //SmartDashboard.putNumber("accel", gyro.getWorldLinearAccelX());
     

    }
  
    @Override
    public void disabledInit() {}

     public void ButtonScoringZones() {
    if (one.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.H;
    }
    else if (two.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.G;
    }
    else if (three.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.F;
    }
    else if (four.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.E;
    }
    else if (five.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.D;
    }
    else if (six.getAsBoolean()) {
      CurrnetScoringPosition = RobotContainer.ScoringPosition.C;
    }
    else if (seven.getAsBoolean()) {
      CurrnetScoringPosition= RobotContainer.ScoringPosition.B;
    } 
    else if (eight.getAsBoolean()) {
        CurrnetScoringPosition = RobotContainer.ScoringPosition.A;
    } 
    else if (nine.getAsBoolean()) {
        CurrnetScoringPosition = RobotContainer.ScoringPosition.L;
    } 
    else if (ten.getAsBoolean()) {
        CurrnetScoringPosition = RobotContainer.ScoringPosition.K;
    } 
    else if (eleven.getAsBoolean()) {
        CurrnetScoringPosition = RobotContainer.ScoringPosition.J;
    } 
    else if (twelve.getAsBoolean()) {
        CurrnetScoringPosition = RobotContainer.ScoringPosition.I;
    }
  }

    
public static enum levelscore {
  Level1,
  Level2,
  Level3,
  Level4,

}

public static enum ReefMode {
  ALGAE,
  CORAL
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



 


    public static void setScoringPosition(ScoringPosition position) {
      CurrnetScoringPosition = position;
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

   // m_robotContainer.superstructure.setDesiredState(SuperstructureState.HOME_UP);
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putString("CurrnetScoringPosition", CurrnetLevelPosition.toString());
    SmartDashboard.putNumber("current scoring location X ", RobotState.getInstance().getScoringPose().getMeasureX().magnitude());
    SmartDashboard.putNumber("current scoring location Y ", RobotState.getInstance().getScoringPose().getMeasureY().magnitude());

    SmartDashboard.putString("scoring pose", CurrnetScoringPosition.toString());
    
   ButtonLevelscoring();
   ButtonScoringZones();

   RobotState.getInstance().getAlgaeLevel(m_robotContainer.drive.getEstimatedPosition());
   RobotState.getInstance().getIntakingPose(m_robotContainer.drive.getEstimatedPosition());
   SmartDashboard.putNumber("FMS TIME", DriverStation.getMatchTime());
   
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
