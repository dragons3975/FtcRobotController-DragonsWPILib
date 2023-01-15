package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.CallibrateAscenseurCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class PickUpSequentialCommandGroup extends SequentialCommandGroup {

    public PickUpSequentialCommandGroup(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem) {
        AscenseurCommand positionPickUp = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        CallibrateAscenseurCommand calibrate = new CallibrateAscenseurCommand(telemetry, ascenseurSubsystem);

        addCommands(
                positionPickUp,
                calibrate
            );
    }

}
