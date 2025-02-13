package frc.robot.Subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {


    @AutoLog
    public static class ElevatorIOInputs {
        public double encoderRotations_L = 0.0;
        public double currentOutput_L = 0.0;
    }


    public default void setPosition(double position) {}

    public default void resetPosition() {}

   
    
    public default void updateInputs(ElevatorIOInputs inputs) {}
}
