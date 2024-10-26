package frc.team3128.subsystems;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.lang.Thread.State;
import java.lang.management.MonitorInfo;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake {

    public enum Setpoint {
        //define your enums here (the diff heights you want to place cones at - ie. low pole, mid pole, etc)
        LOWPOLE(0),
        MIDPOLE(45),
        HIGHPOLE(70)
        ;

        public final double angle;
        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    public class IntakePivot extends PivotTemplate {

        private IntakePivot() {

            //define your controller here (using trap controller)
            //setTolerance here (aka the max angle error that's acceptable)
            //setConstraints (min and max position of the intake - in degrees)
            //call initShuffleboard so you can debug using shuffleboard

        }

        @Override  
        protected void configMotors() {
            //define one motor (use NAR_CANspark as the type of motor)
            //motor should have setInverted as false
            //set motor's NeutralMode as COAST
        
        }

        @Override
        public void useOutput(double output, double setpoint) {
           // motor.setVolts(MathUtil.clamp(output, -12, 12));
        }
        
        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(()-> startPID(setpoint.getAsDouble()));
        }

    }
    
    public class IntakeRollers extends ManipulatorTemplate { 

    private DigitalInput limitSwitch;

    private IntakeRollers() {
        super(CURRENT_THRESHHOLD, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, MOTOR);

        /*
        *define constants for all of the above parameters 
        * Creates an Manipulator object.
        *currentThreshold Current when object is intook.
        
        *intakePower Intake power.
        *outtakePower Outtake power.
        *stallPower Stall Power, run when Manipulator has a game piece.
        *lagSeconds Time before current check is run.
        *motors Manipulator motors.
        */
        
        //??????????????????????????????????????limitSwitch = new DigitalInput(7);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        //NAR_CANSpark
        //define one motor (again, using nar_canspark)???
       
        //setInverted as false
        MOTOR.setInverted(false);
        //setNeutralMode as COAST
        MOTOR.setNeutralMode(Neutral.COAST);
        //setCurrentLimit as 40
        MOTOR.setCurrentLimit(40);
        }
    

}

    private static Intake instance;

    public IntakePivot intakePivot;
    public IntakeRollers intakeRollers;

    public boolean isRetracting = false;

    //create a getInstance() so you can always create a new intake 
    //ie. public static synchronized Intake getInstance(){
    public static synchronized Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake(){
        //create a new intakePivot 
        intakePivot = new IntakePivot();
        //create a new intakeRollers
        intakeRollers = new IntakeRollers();
    }

    
    public Command intake(Setpoint setpoint) {
        return sequence(
            deadline(
                //use intakeRollers to call intake(). Remember to add a comma after your sentence (proper format for sequence/parallel commands)
                intakeRollers.intake(),
                sequence(
                    //have your intakePivot pivot to setpoint.angle
                    intakePivot.pivotTo(setpoint.angle),
                    waitUntil(() -> hasObjectPresent()),
                    runOnce(() -> setPower(STALL_POWER))
                )
                )
            );
    }

    public Command outtake() {
        return sequence (
            //outtaking so have your intakePivot pivot to your enum state
            intakePivot.pivotTo(Setpoint.LOWPOLE.angle),
            //have intakeRollers run manipulator at outake power
            intakeRollers.runManipulator(OUTTAKE_POWER)
        );
    }
    
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > CURRENT_THRESHHOLD;
    }
    
    public double getCurrent(){
        return motors.getStallCurrent();
    }
    
}
