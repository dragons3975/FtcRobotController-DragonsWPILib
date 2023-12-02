package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem) {

        ParallelRaceGroup avancer5sec = new DriveAutoCommand(driveSubsystem, 1, 0, 0).withTimeout(5);
        ParallelRaceGroup tourner1sec = new DriveAutoCommand(driveSubsystem, 0, 1, 0).withTimeout(1);
        ParallelRaceGroup reculer5sec = new DriveAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(5);

        addCommands(
            avancer5sec
        );
    }

}
