// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.hardware.motorcontroller.NAR_CANSpark;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {

    public static Alliance alliance;

    public static Alliance getAlliance() {
        if (alliance == null) {
            Optional<Alliance> DSalliance = DriverStation.getAlliance();
            if (DSalliance.isPresent()) alliance = DSalliance.get();
        }
        return alliance;
    }

    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    public static AutoPrograms autoPrograms;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        autoPrograms = new AutoPrograms();
        m_robotContainer.initDashboard();
        LiveWindow.disableAllTelemetry();
        if (NAR_CANSpark.getNumFailedConfigs() > 0) {
            Leds.getInstance().setLedColor(Colors.ERROR);
        }
        else {
            Leds.getInstance().setLedColor(Colors.CONFIGURED);
        }

        // addReceiver(true, LoggingState.NONE);
    }

    @Override
    public void robotPeriodic(){
        Camera.updateAll();
    }

    @Override
    public void autonomousInit() {
        Leds.getInstance().setDefaultColor();
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        // if(DriverStation.getMatchType() != MatchType.None)
        //     addReceiver(true, LoggingState.FULLMATCH);
        //Logger.start();

    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        // if(DriverStation.getMatchType() != MatchType.None)
        //     addReceiver(true, LoggingState.FULLMATCH);
        // Logger.start();
        
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Swerve.getInstance().setBrakeMode(true);
        CommandScheduler.getInstance().cancelAll();
        sequence(
            waitSeconds(3.0).ignoringDisable(true),
            runOnce(()->Swerve.getInstance().setBrakeMode(false)).ignoringDisable(true)
        ).schedule();
    }

    @Override
    public void disabledExit() {
        Swerve.getInstance().setBrakeMode(false);
    }
    
    @Override
    public void disabledPeriodic() {

    }
}
