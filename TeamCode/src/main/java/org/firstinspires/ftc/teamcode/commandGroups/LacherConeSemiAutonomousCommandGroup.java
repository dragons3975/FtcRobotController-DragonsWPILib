package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class LacherConeSemiAutonomousCommandGroup extends SequentialCommandGroup {

    public LacherConeSemiAutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem) {

        DriveAutoCommand avancer40cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        DriveAutoCommand reculer30cm = new DriveAutoCommand(telemetry, driveSubsystem, -30, 0);
        DriveAutoCommand reculer10cm = new DriveAutoCommand(telemetry, driveSubsystem, -10, 0);

        ParallelRaceGroup ouvrirPinceWithTimeout = new OuvrirPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);
        ParallelRaceGroup fermerPinceWithTimeout = new FermerPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);

        addCommands(
                avancer40cm,
                reculer10cm,
                ouvrirPinceWithTimeout,
                reculer30cm,
                fermerPinceWithTimeout
        );
    }

}
