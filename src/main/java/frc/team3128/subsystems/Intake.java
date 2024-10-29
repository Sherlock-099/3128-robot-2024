package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
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
import frc.team3128.Constants.IntakeConstants;

public class Intake {

    public enum Setpoint {
        // define your enums here (the diff heights you want to place cones at - ie. low
        // pole, mid pole, etc)
        ;

        public final double angle;

        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    public class IntakePivot extends PivotTemplate {

        private IntakePivot() {

            // define your controller here (using trap controller)
            // setTolerance here (aka the max angle error that's acceptable)
            // setConstraints (min and max position of the intake - in degrees)
            // call initShuffleboard so you can debug using shuffleboard

        }

        @Override
        protected void configMotors() {
            // define one motor (use NAR_CANspark as the type of motor)
            // motor should have setInverted as false
            // set motor's NeutralMode as COAST

        }

        @Override
        public void useOutput(double output, double setpoint) {
            // motor.setVolts(MathUtil.clamp(output, -12, 12));
        }

        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(() -> startPID(setpoint.getAsDouble()));
        }

    }

    public class IntakeRollers extends ManipulatorTemplate {

        private IntakeRollers() {
            super(IntakeConstants.STALL_CURRENT, IntakeConstants.INTAKE_POWER, IntakeConstants.OUTTAKE_POWER,
                    IntakeConstants.STALL_POWER, 0.3, IntakeConstants.RIGHT_ROLLER_MOTOR);
            /*
             * define constants for all of the above parameters
             * Creates an Manipulator object.
             * currentThreshold Current when object is intook.
             * intakePower Intake power.
             * outtakePower Outtake power.
             * stallPower Stall Power, run when Manipulator has a game piece.
             * lagSeconds Time before current check is run.
             * motors Manipulator motors.
             */

            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            // define one motor (again, using nar_canspark)
            IntakeConstants.RIGHT_ROLLER_MOTOR.setInverted(false);
            // setNeutralMode as COAST
            IntakeConstants.RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            // setCurrentLimit as 40
            IntakeConstants.RIGHT_ROLLER_MOTOR.setCurrentLimit(40);
        }

    }

    private static Intake instance;

    public IntakePivot intakePivot;
    public IntakeRollers intakeRollers;

    public boolean isRetracting = false;

    // create a getInstance() so you can always create a new intake
    // ie. public static synchronized Intake getInstance(){
    public static synchronized Intake getInstance(){

    }


    private Intake() {
        // create a new intakePivot
        intakePivot = new IntakePivot();
        // create a new intakeRollers
        intakeRollers = new IntakeRollers();
    }

    public Command intake(Setpoint setpoint) {
        return sequence(
                deadline(
                        // use intakeRollers to call intake(). Remember to add a comma after your
                        // sentence (proper format for sequence/parallel commands)
                        intakeRollers.intake(),
                        sequence(
                                // have your intakePivot pivot to setpoint.angle
                                intakePivot.pivotTo(setpoint.angle),
                                waitUntil(() -> hasObjectPresent()),
                                runOnce(() -> setPower(IntakeConstants.STALL_POWER)))));
    }

    public Command outtake(Setpoint setpoint) {
        return sequence(
                // outtaking so have your intakePivot pivot to your enum state
                intakePivot.pivotTo(setpoint.angle),
                // have intakeRollers run manipulator at outake power
                intakeRollers.runManipulator(IntakeConstants.OUTTAKE_POWER));
    }

    public boolean hasObjectPresent() {
        return Math.abs(getCurrent()) > IntakeConstants.STALL_CURRENT;
    }

    public double getCurrent() {
        return motors.getStallCurrent();
    }

}
