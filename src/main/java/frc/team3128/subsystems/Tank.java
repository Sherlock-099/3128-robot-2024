package frc.team3128.subsystems;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.RobotContainer;

public class Tank extends SubsystemBase {
    private NAR_Motor leftMotor;
    private NAR_Motor rightMotor;
    private DifferentialDrive drive;

    public Tank(NAR_Motor leftMotor, NAR_Motor rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.drive = new DifferentialDrive(leftMotor.getMotor(), rightMotor.getMotor());
    }

    public void tankDrive(double leftSpeed, double rightSpeed){
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void setPower(double leftVolts, double rightVolts){
        tankDrive(leftVolts/ RobotController.getBatteryVoltage(), rightVolts/ RobotController.getBatteryVoltage());
    }

    public void stop(){
        tankDrive(0,0);
    }
}
