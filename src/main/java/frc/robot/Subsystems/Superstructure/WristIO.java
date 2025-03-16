package frc.robot.Subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Subsystems.Superstructure.Superstructure.ManualMode;

public interface WristIO {


    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        //haven't reset
        public boolean sensorBoolean = false;

        public double armAngle = 0.0;

        public double pivotEncoderAbs = 0.0;

        public double current = 0.0;
        //public boolean reached_upper_limit = false; 
    }

    default void setWristPositionSLOW(double angle) {
    }

    default void setVerticalAngleSLOW(double angle) {
    }


    default void updateInputs(WristIOInputs inputs, ManualMode mode) {
    }

    default void resetPivot() {}


    default void resetWrist() {

    }
    default void setWristPosition(double angle) {
    }

    default void setVerticalAngle(double angle) {
    }

    default void setOutputOpenLoop(double output) {
    }

    default void setOutputOpenLoopPivot(double output) {
       
    }

    default void setOutputOpenLoopWrist(double output) {
    }

    default void keepStill() {}


    
}
