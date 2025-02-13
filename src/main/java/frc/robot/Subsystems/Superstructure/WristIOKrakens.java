package frc.robot.Subsystems.Superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.WristConstants;

public class WristIOKrakens implements WristIO {
    private TalonFX pivotMotor = new TalonFX(WristConstants.ID_Pivot, "rio");
    private SparkFlex wristMotor = new SparkFlex(WristConstants.ID_Wrist, MotorType.kBrushless);
    private SparkFlex armIntakeMotor = new SparkFlex(WristConstants.ID_Intake, MotorType.kBrushless);
    //make sure channel match roborio channels
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    //put actual port for it
    private DigitalInput limitSwitch = new DigitalInput(0);

   
    private PositionVoltage positionRequester = new PositionVoltage(0);

     
    public WristIOKrakens() {
        //PIVOT CONFIG

        //change based on bot
        armEncoder.setInverted(true);
        double initPos = armEncoder.get();
        Slot0Configs slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine).withKP(WristConstants.kP).withKG(WristConstants.kG).withKV(WristConstants.kV).withKA(WristConstants.kA);
        TalonFXConfiguration config_pivot = new TalonFXConfiguration();
        config_pivot.Slot0 = slot0Configs;
        config_pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config_pivot.Feedback.RotorToSensorRatio = WristConstants.rotorToSensorRatio;
        config_pivot.MotionMagic.MotionMagicAcceleration = WristConstants.AccelerationMotionMagic;
        config_pivot.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CruisingVelocityMotionMagic;

        pivotMotor.getConfigurator().apply(config_pivot);
        //offset = reading at 0 degrees
        //motor reading = encoder reading - offset
        pivotMotor.setPosition(initPos - WristConstants.offsetPivot);

        //Turret Config

        SparkFlexConfig config_wrist = new SparkFlexConfig();
       config_wristg_wrist.closedLoop.p(WristConstants.kP_wrist);
    


    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.armAngle = armEncoder.get();
        inputs.sensorBoolean = se
    }

    public void setAngle(double angle) {
        pivotMotor.setControl(positionRequester.withPosition(angle).withEnableFOC(true).withFeedForward());
    }
    
}
