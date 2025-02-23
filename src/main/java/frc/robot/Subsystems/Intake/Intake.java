package frc.robot.Subsystems.Intake;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{
    public static enum Intake_states {
        Bofore_First,
        After_First,
        Ready,
        Empty,
        Intermediate
    }
    public Intake_states CurrentintakeState = Intake_states.Empty; // or maybe here we need to put first
    SparkMax BoxMotor = new SparkMax(18, MotorType.kBrushless);
    SparkMax funnel = new SparkMax(21, MotorType.kBrushless);
    double initTime = 0;
   
    
    //---------------------------------------------------------Motors spining speeds--------------------------------------------------------
    public void before_first(){
        
        funnel.set(0.15);
        BoxMotor.set(0.0);
        
    }



    public void intermediate(){
        funnel.set(0.15);
        BoxMotor.set(0.2);
        
    }

    public void after_first() {
        funnel.set(0.15);
        BoxMotor.set(0.15);
    }
    
    public void ReadyBOT() {
       
        BoxMotor.set(0);
        funnel.set(0);
    }
    public void EmptyBOT() {
        
        BoxMotor.set(0);
        funnel.set(0);
    }




    //--------------------------------------------------------------------------------------------------------------------------------------
    
    DigitalInput DistanceSensorOne = new DigitalInput(1); //first sensor
    DigitalInput DistanceSensorTwo = new DigitalInput(2); //second sensor
    DigitalInput DistanceSensorThree = new DigitalInput(3); //third sensor
    /* the order is 
     * first sensor
     * third sensor
     * second sensor
     */
    // switch the second and third and the end!! <--------------------
    @Override
    public void periodic() {
        SmartDashboard.putString("State", CurrentintakeState.toString());
         if (CurrentintakeState.equals(Intake_states.Bofore_First) && !DistanceSensorOne.get()) {
            CurrentintakeState = Intake_states.After_First; 
            // here we need to turn off the blue wheels spin the ninja stars and get the elevator up
        
        }
        // if (CurrentintakeState.equals(Intake_states.Bofore_First) && !DistanceSensorOne.get()){
        //     CurrentintakeState = Intake_states.After_First; 
        //     // here we need to turn off the blue wheels spin the ninja stars and get the elevator up
        
        // }


        else if (CurrentintakeState.equals(Intake_states.Intermediate) && !DistanceSensorThree.get() && !DistanceSensorTwo.get()) {
            CurrentintakeState = Intake_states.Ready;
            // here we need to stop the ninja stars and the elevator
        }
        else if (CurrentintakeState.equals(Intake_states.Ready) && DistanceSensorTwo.get() && (DistanceSensorTwo.get() | DistanceSensorThree.get())) {
            CurrentintakeState = Intake_states.Empty;
            // here we need to stop the ninja stars and the elevator
        }

        else if (CurrentintakeState.equals(Intake_states.After_First) && (!DistanceSensorThree.get() || !DistanceSensorTwo.get())) {
            CurrentintakeState = Intake_states.Intermediate;
           
        }

        else if (CurrentintakeState.equals(Intake_states.Empty) && !DistanceSensorTwo.get() && !DistanceSensorThree.get()) {
            CurrentintakeState = Intake_states.Ready;
        }


        
    

        
       





        if ( CurrentintakeState.equals(Intake_states.After_First)) {
            after_first();
        }
       
        else if ( CurrentintakeState.equals(Intake_states.Ready)) {
            ReadyBOT();
        }
        else if ( CurrentintakeState.equals(Intake_states.Empty)) {
            EmptyBOT();
        }

        else if (CurrentintakeState.equals(Intake_states.Bofore_First)) {
            before_first();     
        }

        else if (CurrentintakeState.equals(Intake_states.Intermediate)) {
            intermediate();
        }
        
    }
    

    public void setState(Intake_states new_intake_state) {
        CurrentintakeState =new_intake_state;
    }
    public Intake_states getstate() {
        return CurrentintakeState;
    }
    // if intake while press is on then top wheels spin --*--Empty_state is on--*--
    // when you get the first distance sensor on you can turn the funnel motor off and elevator goes up for the intake --*--After_first state is on--*--
    // when you hit the third distance sensor you start the shuffle --*--suffle_state is on--*--
    // when you hit the second and the first one it needs to stop the intake --*--Ready_state is on--*--    
}
