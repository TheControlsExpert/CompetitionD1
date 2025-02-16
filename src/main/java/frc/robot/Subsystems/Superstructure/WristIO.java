package frc.robot.Subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {


    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        //haven't reset
        public boolean sensorBoolean = false;

        public double armAngle = 0.0;

        public double current = 0.0;
        //public boolean reached_upper_limit = false; 
    }


    default void updateInputs(WristIOInputs inputs) {
    }

    default void setWristPosition(double angle) {
    }

    default void setVerticalAngle(double angle) {
    }

    default void setOutputOpenLoop(double output) {
    }



    
}
