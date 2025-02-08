package frc.robot.Subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {


    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        public boolean wristZeroed = false;
        public double armAngle = 0.0;
        public boolean reached_upper_limit = false; 
    }


    default void IntakeBox() {
    }

    default void IntakeGround() {
    }

    default void Score() {
    }

    default void setAngle(double angle) {

    }

    
}
