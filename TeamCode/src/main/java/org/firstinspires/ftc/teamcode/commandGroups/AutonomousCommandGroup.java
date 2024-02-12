package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem) {

        //ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        ParallelRaceGroup avance = new AvanceAutoCommand(driveSubsystem, 0, 1, 0).withTimeout(3);
        ParallelRaceGroup tasse = new AvanceAutoCommand(driveSubsystem, 1, 0, 0).withTimeout(3);
        ParallelRaceGroup tourne = new AvanceAutoCommand(driveSubsystem, 0, 0, 1).withTimeout(3);


        addCommands(
                avance,
                tasse,
                tourne
                //calibration
                );
    }

}
