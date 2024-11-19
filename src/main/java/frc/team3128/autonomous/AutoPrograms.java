package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.HashMap;
import frc.team3128.subsystems.Intake;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {
        private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private HashMap<String, Command> pathMap = new HashMap<String, Command>();

    public AutoPrograms() {
        Intake.getInstance();
        Trajectories.initTrajectories();
        initAutoSelector();
    }


    private void initAutoSelector() {
        final String[] autoStrings = new String[] {
            "default",
            "topFar_4note",
            "topFarCopy_4note",
            "middleClose_4note",
            "middleClose_5note",
            "middle_6note",
            "bottom_7note",
            "special_3note",
        };
        final String[] pathStrings = new String[] {
            "only-note1.2-note2.3",
            "only-note2.3-middle",
            "only-top-note2.1",
            "only-note2.1-wing",
            "only-wing-note2.2",
            "only-note2.2-wing",
            //"only-wing-note2.3",
            "only-note2.3-wing"
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            if (auto.equals("default")) continue;
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
        for (String path : pathStrings) {
            pathMap.put(path, Trajectories.getPathPlannerAuto(path));
        }
    }
    public Command getAuto(String name) {
        return autoMap.get(name);
    }

    public Command getPath(String name) {
        return pathMap.get(name);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        String hardcode = "middleClose_4note";
        
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        autoCommand = autoMap.get(selectedAutoName);

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }

    private Command defaultAuto(){
        return none();
        // sequence(
        //         Trajectories.resetAuto(),
        //         waitSeconds(5),
        //         autoShoot(),
        //         waitSeconds(2),
        //         run(()-> Swerve.getInstance().drive(new Translation2d(Robot.getAlliance() == Alliance.Blue ? 1 : -1, 0), 0, true))
        //     );
    }
}