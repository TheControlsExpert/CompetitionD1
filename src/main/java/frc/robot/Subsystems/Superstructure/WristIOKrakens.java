package frc.robot.Subsystems.Superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;
import frc.robot.util.DoubleProfilerArmNeo;

public class WristIOKrakens implements WristIO {
    private SparkFlex pivotMotor = new SparkFlex(15, MotorType.kBrushless);
    private SparkFlex wristMotor = new SparkFlex(16, MotorType.kBrushless);
    private SparkFlex armIntakeMotor = new SparkFlex(17, MotorType.kBrushless);
    //make sure channel match roborio channels
    private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    //put actual port for it
   // private DigitalInput limitSwitch = new DigitalInput(0);
    double setpointangle = 17.5;
    ProfiledPIDController profiler_racial = new ProfiledPIDController(0.005, 0, 0, new Constraints(300 * 10/60, 200 * 6/60));
    DoubleProfilerArmNeo profiler_new = new DoubleProfilerArmNeo(profiler_racial, pivotMotor);

    private double output = 0.0;

    double kv = 0.003;
    double ks = 0.017;
    double offset = 0.455;
    double kg = 0.017;

   
    //private PositionVoltage positionRequester = new PositionVoltage(0);
    

     
    public WristIOKrakens() {
        //PIVOT CONFIG

        //change based on bot

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.apply(new ClosedLoopConfig().p(0.000));
       // config.


        pivotMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotMotor.getEncoder().setPosition((armEncoder.get() - offset) * 75);
       

       

        //Turret Config

        SparkFlexConfig config_wrist = new SparkFlexConfig();
        config_wrist.closedLoop.p(0.15);
        config_wrist.idleMode(IdleMode.kBrake);
        config_wrist.inverted(false);

        wristMotor.configure(config_wrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        wristMotor.getEncoder().setPosition(0);

        //intake config

        armIntakeMotor.configure(new SparkFlexConfig().idleMode(IdleMode.kCoast).inverted(false), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


          
    


    }

    public void updateInputs(WristIOInputs inputs, ManualMode mode) {
        SmartDashboard.putNumber("output duty cycle", output);
        SmartDashboard.putString("actual seen mode by arm", mode.toString());

        inputs.armAngle = pivotMotor.getEncoder().getPosition();
        //inputs.sensorBoolean = limitSwitch.get();
        inputs.wristAngle = wristMotor.getEncoder().getPosition();
        inputs.current = armIntakeMotor.getOutputCurrent();
        inputs.pivotEncoderAbs = armEncoder.get() - offset;

        SmartDashboard.putNumber("Arm current", inputs.current);

        SmartDashboard.putNumber("Arm Angle Theoretical", inputs.armAngle);
        SmartDashboard.putNumber("Arm angle actual", pivotMotor.getEncoder().getPosition());

        if (mode.equals(ManualMode.AUTOMATIC)) {
            SmartDashboard.putBoolean("r we doing automatic for the arm", true);
        double setpoint = profiler_racial.calculate(pivotMotor.getEncoder().getPosition(), setpointangle);
        double setpoint2 = profiler_new.calculate(setpointangle);
   
    SmartDashboard.putNumber("setpoint",  profiler_racial.getSetpoint().velocity * 60 );
    SmartDashboard.putNumber("setpoint angle is act set", setpointangle );
   // SmartDashboard.putNumber("velocity pivot",  pivot.getEncoder().getVelocity());
   
       //  motorL.set(0.05);
        pivotMotor.getClosedLoopController().setReference(profiler_racial.getSetpoint().velocity * 60, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 12 * (Math.signum(profiler_racial.getSetpoint().velocity) * ks)  +  12 * (kg * Math.cos(Units.rotationsToRadians(armEncoder.get() - offset))) + kv * profiler_racial.getSetpoint().velocity * 60 + setpoint, ArbFFUnits.kVoltage);
        }

        else {

            if (pivotMotor.getEncoder().getPosition() > 22 || pivotMotor.getEncoder().getPosition() < -22) {
                output = 0;
            }
            SmartDashboard.putBoolean("r we doing automatic for the arm bubbles", false);

            pivotMotor.getClosedLoopController().setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0,  12 * (kg * Math.cos(Units.rotationsToRadians(armEncoder.get() - offset))) + output, ArbFFUnits.kVoltage);


        }

    }

    public void setVerticalAngle(double angle) {

        setpointangle = angle;
      
        // motorL.setControl(velocitycreator.withPosition(i));
        // pivot.set(i);
   
        //motorL.set(0.3);
    }

    public void setOutputOpenLoopPivot(double output) {
        this.output = output;
    }

    public void setOutputOpenLoopWrist(double output) {
        wristMotor.set(output);
    }

    public void setWristPosition(double angle) {
        wristMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
    }

    public void setOutputOpenLoop(double output) {
        //use to reset position
        armIntakeMotor.set(output);
    }

    public void setIntakeOutput(double output) {
        armIntakeMotor.set(output);
    }

    
    
}
