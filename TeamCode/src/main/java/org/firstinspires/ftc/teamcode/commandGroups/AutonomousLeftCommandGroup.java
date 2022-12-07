package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousLeftCommandGroup extends SequentialCommandGroup {

    public AutonomousLeftCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        DriveAutoCommand avancer40Cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);

            addCommands(
                    avancer40Cm
            );
    }

}
