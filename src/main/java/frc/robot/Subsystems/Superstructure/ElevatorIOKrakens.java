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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;
import frc.robot.util.CTREDoubleProfilerElevator;

public class ElevatorIOKrakens implements ElevatorIO {
        TalonFX motorL = new TalonFX(20, "rio");
        TalonFX motorR = new TalonFX(14, "rio");
        double setpointPosition = 2;
        double timer = -10000;
        double fullTimer = -10000;
        boolean slow = false;
       // double fulltimesincesetpointchange = -10000;
        //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, 0);
        // VelocityVoltage velocityRequester = new VelocityVoltage(0);
        final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);
         CTREDoubleProfilerElevator profiler = new CTREDoubleProfilerElevator(motorL);
        double encoderPos = 0.0;
        


    public ElevatorIOKrakens() {
        TalonFXConfiguration configL = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs().withKV(0.12).withKS(0.0115* 12 ).withKG(0.0295 * 12).withKP(0.03).withKA(0).withGravityType(GravityTypeValue.Elevator_Static);
        
        var motionMagicConfigs = configL.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 100;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        configL.Slot0 = slot0;
        configL.MotorOutput.NeutralMode = NeutralModeValue.Brake;  
        configL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        
        motorL.getConfigurator().apply(configL);

      
      
      
        TalonFXConfiguration configR = new TalonFXConfiguration();
        configR.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //tune this to the bot
        configR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorR.getConfigurator().apply(configR);
        motorR.setControl(new StrictFollower(20));
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.apply(new ClosedLoopConfig().p(0.0001));
       // config.
        motorL.setPosition(0);
        motorR.setPosition(0);
      
    }



    @Override
    public void updateInputs(ElevatorIOInputs inputs, ManualMode mode) {
        
        inputs.encoderRotations_L = motorL.getPosition().getValueAsDouble();
        encoderPos = inputs.encoderRotations_L;
        inputs.currentOutput_L = motorL.getStatorCurrent().getValueAsDouble();

        // if (slow) {
        //     motorL.setControl(mm_request.)
        // }


        SmartDashboard.putNumber("elevator speed", motorL.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("setpoint tracker for elevator", setpointPosition);

        if (mode.equals(ManualMode.AUTOMATIC)) {

        if (Timer.getFPGATimestamp() - timer > 1.0 && timer > 0 && Math.abs(motorL.getPosition().getValueAsDouble() - setpointPosition) > 1 && Math.abs(motorL.getVelocity().getValueAsDouble()) < 0.1) {
            motorL.set(Math.signum(setpointPosition - encoderPos) * 0.1);
            timer = Timer.getFPGATimestamp();
        }

        else {
            motorL.setControl(mm_request.withPosition(setpointPosition));
            SmartDashboard.putBoolean("elevatorTracking", true);

        }

        
    

        if (Timer.getFPGATimestamp() - fullTimer > 8 && fullTimer > 0 && Math.abs(motorL.getPosition().getValueAsDouble() - setpointPosition) > 1) {
            SmartDashboard.putBoolean("elevatorTracking", false);
        }
    }

    else {
        timer = -10000;
        fullTimer = -10000;


    }

        
        
    }


    @Override
    public void setPosition(double position) {
        slow = false;
        if (DriverStation.isEnabled()) {
            if (position != setpointPosition) {
                timer = Timer.getFPGATimestamp();
                fullTimer = Timer.getFPGATimestamp();
            }
         setpointPosition = position;

        // //assumes positive voltage results in moving up
         

        //profiler.runProfile(position, mm_request);
        }
        

        
    }

    // public void setPositionSLOW(double position) {
    //     slow = true;
    //     if (DriverStation.isEnabled()) {
    //         if (position != setpointPosition) {
    //             timer = Timer.getFPGATimestamp();
    //             fullTimer = Timer.getFPGATimestamp();
    //         }
    //      setpointPosition = position;

    //     // //assumes positive voltage results in moving up
         

    //     //profiler.runProfile(position, mm_request);
    //     }
    // }



    public void stop() {
        motorL.setControl(new DutyCycleOut(0.029));
    }


    public void resetPosition() {
        motorL.setPosition(0);
        motorR.setPosition(0);

    }

    public void setOutputOpenLoop(double output) {
      motorL.set(output);
    }

    public double getEncoderSpeed() {
        return motorL.getVelocity().getValueAsDouble();
    }

    






    
}
