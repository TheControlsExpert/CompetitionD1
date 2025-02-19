// package frc.robot.Subsystems.Superstructure;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.Constants.WristConstants;
// import frc.robot.util.DoubleProfilerArmNeo;

// public class WristIOKrakens implements WristIO {
//     private SparkFlex pivotMotor = new SparkFlex(WristConstants.ID_Pivot, MotorType.kBrushless);
//     private SparkFlex wristMotor = new SparkFlex(WristConstants.ID_Wrist, MotorType.kBrushless);
//     private SparkFlex armIntakeMotor = new SparkFlex(WristConstants.ID_Intake, MotorType.kBrushless);
//     //make sure channel match roborio channels
//     private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
//     //put actual port for it
//     private DigitalInput limitSwitch = new DigitalInput(0);
    
//     ProfiledPIDController profiler_racial = new ProfiledPIDController(0.005, 0, 0, new Constraints(300 * 10/60, 200 * 6/60));
//     DoubleProfilerArmNeo profiler_new = new DoubleProfilerArmNeo(profiler_racial, pivotMotor);



//     double kv = 0.003;
//     double ks = 0.017;
//     double offset = 0.455;
//     double kg = 0.019;

   
//     //private PositionVoltage positionRequester = new PositionVoltage(0);
    

     
//     public WristIOKrakens() {
//         //PIVOT CONFIG

//         //change based on bot
//         armEncoder.setInverted(true);
//         double initPos = armEncoder.get();
//         Slot0Configs slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine).withKP(WristConstants.kP).withKG(WristConstants.kG).withKV(WristConstants.kV).withKA(WristConstants.kA);
//         TalonFXConfiguration config_pivot = new TalonFXConfiguration();
//         config_pivot.Slot0 = slot0Configs;
//         config_pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         config_pivot.Feedback.RotorToSensorRatio = WristConstants.rotorToSensorRatio;
//         config_pivot.MotionMagic.MotionMagicAcceleration = WristConstants.AccelerationMotionMagic;
//         config_pivot.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CruisingVelocityMotionMagic;

      

//         //Turret Config

//         SparkFlexConfig config_wrist = new SparkFlexConfig();
//         config_wrist.closedLoop.p(WristConstants.kP_wrist);
//         config_wrist.idleMode(IdleMode.kBrake);
//         config_wrist.inverted(false);

//         wristMotor.configure(config_wrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//         //intake config

//         armIntakeMotor.configure(new SparkFlexConfig().idleMode(IdleMode.kCoast).inverted(false), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


          
    


//     }

//     public void updateInputs(WristIOInputs inputs) {
//         inputs.armAngle = armEncoder.get();
//         inputs.sensorBoolean = limitSwitch.get();
//         inputs.wristAngle = wristMotor.getEncoder().getPosition();
//         inputs.current = armIntakeMotor.getOutputCurrent();
//     }

//     public void setVerticalAngle(double angle) {
//         pivotMotor.setControl(pivotMagicController.withPosition(angle).withEnableFOC(true));
//     }

//     public void setWristPosition(double angle) {
//         wristMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
//     }

//     public void setOutputOpenLoop(double output) {
//         //use to reset position
//         wristMotor.set(output);
//     }

//     public void setIntakeOutput(double output) {
//         armIntakeMotor.set(output);
//     }

    
    
// }
