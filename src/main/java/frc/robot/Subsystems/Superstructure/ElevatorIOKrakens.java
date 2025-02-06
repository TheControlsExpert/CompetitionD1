// package frc.robot.Subsystems.Superstructure;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.StrictFollower;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// public class ElevatorIOKrakens implements ElevatorIO {
//     TalonFX krakenLeft = new TalonFX(ElevatorConstants.IDLeft, "rio");
//     TalonFX krakenRight = new TalonFX(ElevatorConstants.IDRight, "rio");
//     SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, 0);
//     VelocityVoltage speedRequester = new VelocityVoltage(0);


//     public ElevatorIOKrakens() {
//         TalonFXConfiguration configL = new TalonFXConfiguration();
//         Slot0Configs slot0 = new Slot0Configs().withKP(ElevatorConstants.kP).withKD(ElevatorConstants.kD);  
//         configL.Slot0 = slot0;
//         configL.MotorOutput.NeutralMode = NeutralModeValue.Brake;  
        
//         krakenLeft.getConfigurator().apply(configL);

//         TalonFXConfiguration configR = new TalonFXConfiguration();
//         configR.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         //tune this to the bot
//         configR.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//         krakenRight.getConfigurator().apply(configR);
//         krakenRight.setControl(new StrictFollower(ElevatorConstants.IDLeft));
//     }



//     @Override
//     public void updateInputs(ElevatorIOInputs inputs) {
//         inputs.encoderRotations_L = krakenLeft.getPosition().getValueAsDouble();
//         inputs.currentOutput_L = krakenLeft.getStatorCurrent().getValueAsDouble();
//     }


//     @Override
//     public void setVelocity(double velocity) {
//         krakenLeft.setControl(speedRequester.withVelocity(velocity));
//     }

    






    
// }
