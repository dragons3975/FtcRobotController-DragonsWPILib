package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem) {
        //x =  forward-backward
        //y = sideways

        DriveAutoCommand avancer40Cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        DriveAutoCommand tourner20cm = new DriveAutoCommand(telemetry, driveSubsystem, 0, 20);
        DriveAutoCommand reculer20cm = new DriveAutoCommand(telemetry, driveSubsystem, -20, 0);






        addCommands(
                    avancer40Cm,
                tourner20cm,
                avancer40Cm,
                reculer20cm
            );
    }

}
