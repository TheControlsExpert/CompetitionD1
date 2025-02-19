package frc.robot.util;



import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;

public class DoubleProfilerArmNeo {
    private ProfiledPIDController controller;
    double initTime = 0;
    SparkFlex motor;
    double prevvsetpoint = 0;
    boolean hasReset = false;
    double lastResetTime = Timer.getFPGATimestamp();

    public DoubleProfilerArmNeo(ProfiledPIDController controller, SparkFlex motor) {
        this.controller = controller;
        this.motor = motor;
        
    }

    public void initiate() {
        controller.reset(motor.getEncoder().getPosition(), 0);
        initTime = Timer.getFPGATimestamp();
        hasReset = false;
        
    }

    // public boolean shouldUseBiggerkV( double setpoint) {
    //     return 
    // }


    



    public double calculate(double setpoint) {
        if (Timer.getFPGATimestamp() - lastResetTime > 0.5) {
            initiate();
        }
        lastResetTime  = Timer.getFPGATimestamp();

        
        if (prevvsetpoint != setpoint) {
           initTime = Timer.getFPGATimestamp();
           hasReset = false;
            prevvsetpoint = setpoint;
            

        }

        if (Timer.getFPGATimestamp() - initTime > 0.5 && Math.abs(motor.getEncoder().getVelocity()) < 0.25 * 60  && !hasReset) {
            controller.reset(motor.getEncoder().getPosition(), 0);
            hasReset = true;
           
        }

    
        return controller.calculate(motor.getEncoder().getPosition(), setpoint);
    }
    
}
