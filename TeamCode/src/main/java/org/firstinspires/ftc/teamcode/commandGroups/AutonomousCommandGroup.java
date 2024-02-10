package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        BrasCommandPos1 monteUnPeu = new BrasCommandPos1(brasSubsystem, -80);
        ParallelRaceGroup avancer5sec = new DriveAutoCommand(driveSubsystem, 0, -1, 0).withTimeout(2);
        PinceAutoCommandOuvre ouvre = new PinceAutoCommandOuvre(pinceSubsystem);

        addCommands(
                calibration,
                monteUnPeu,
                avancer5sec,
                ouvre
                );
    }

}
