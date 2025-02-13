package frc.robot.Subsystems.Superstructure;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOKrakens implements ElevatorIO {
        TalonFX krakenLeft = new TalonFX(ElevatorConstants.IDLeft, "rio");
        TalonFX krakenRight = new TalonFX(ElevatorConstants.IDRight, "rio");
        //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, 0);
        // VelocityVoltage velocityRequester = new VelocityVoltage(0);
        final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);
        double encoderPos = 0.0;
        


    public ElevatorIOKrakens() {
        TalonFXConfiguration configL = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs().withKV(ElevatorConstants.kV).withKG(ElevatorConstants.kG).withKP(ElevatorConstants.kP).withKA(ElevatorConstants.kA); 
        var motionMagicConfigs = configL.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.TargetAcceleration;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.TargetVelocity;
        configL.Slot0 = slot0;
        configL.MotorOutput.NeutralMode = NeutralModeValue.Brake;  
        
        krakenLeft.getConfigurator().apply(configL);

        TalonFXConfiguration configR = new TalonFXConfiguration();
        configR.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //tune this to the bot
        configR.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        krakenRight.getConfigurator().apply(configR);
        krakenRight.setControl(new StrictFollower(ElevatorConstants.IDLeft));
    }



    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.encoderRotations_L = krakenLeft.getPosition().getValueAsDouble();
        encoderPos = inputs.encoderRotations_L;
        
        inputs.currentOutput_L = krakenLeft.getStatorCurrent().getValueAsDouble();
    }


    @Override
    public void setPosition(double position) {
        //assumes positive voltage results in moving up
        krakenLeft.setControl(mm_request.withPosition(position).withEnableFOC(true));
        

        
    }

    public void stop() {
        krakenLeft.setControl(new DutyCycleOut(0));
    }


    public void resetPosition() {
        krakenLeft.setPosition(0);
        krakenRight.setPosition(0);

    }

    






    
}
