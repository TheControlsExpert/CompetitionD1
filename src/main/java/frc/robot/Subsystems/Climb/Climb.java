package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    TalonFX ClimbingMotor = new TalonFX(19);
    TalonFXConfiguration motor2_config = new TalonFXConfiguration();
    

    public Climb() {
        motor2_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor2_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ClimbingMotor.getConfigurator().apply(motor2_config);

        ClimbingMotor.setPosition(0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder climb", ClimbingMotor.getPosition().getValueAsDouble());
    }

    public void Climb() {
        // if (ClimbingMotor.getPosition().getValueAsDouble() > 50) {
        //     ClimbingMotor.set(0.0);
        // }

        // else {
        ClimbingMotor.set(1);
        // }



    }
    public void unClimb() {
     //   if (ClimbingMotor.getPosition().getValueAsDouble() < -275) {
        //    ClimbingMotor.set(-0.05);
      //  }

      //  else {
        ClimbingMotor.set(-1);
       // }

    }
    public void Climb_Stop_wCage() {
        ClimbingMotor.set(0.05);
    }
    public void Climb_Stop() {
        ClimbingMotor.set(0.0);
    }
}
