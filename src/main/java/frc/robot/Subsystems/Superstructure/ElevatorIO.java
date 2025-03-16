package frc.robot.Subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;

public interface ElevatorIO {


    @AutoLog
    public static class ElevatorIOInputs {
        public double encoderRotations_L = 0.0;
        public double currentOutput_L = 0.0;
        
    }


    public default void setPosition(double position) {}

    public default void setPositionSLOW(double position) {}

    public default void resetPosition() {}

    public default double getEncoderSpeed() {
        return 0.0;
    }
    
    public default void updateInputs(ElevatorIOInputs inputs, ManualMode mode) {}

    public default void setOutputOpenLoop(double output) {}
}
