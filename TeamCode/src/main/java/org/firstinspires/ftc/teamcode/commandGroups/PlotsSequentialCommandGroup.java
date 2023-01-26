package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.AscenseurSecurityCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class PlotsSequentialCommandGroup extends SequentialCommandGroup {

    public PlotsSequentialCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, AscenseurSubsystem ascenseurSubsystem) {

        DriveAutoCommand avancer40cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        DriveAutoCommand reculer30cm = new DriveAutoCommand(telemetry, driveSubsystem, -30, 0);

        ParallelRaceGroup ouvrirPinceWithTimeout = new OuvrirPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);
        ParallelRaceGroup fermerPinceWithTimeout = new FermerPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);

        AscenseurCommand remonterAscenseur = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        AscenseurCommand baisserAscenseur5plots = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionSecurity);
        AscenseurSecurityCommand groundWithSecurityCommand = new AscenseurSecurityCommand(telemetry, ascenseurSubsystem);

        addCommands(
                baisserAscenseur5plots.alongWith(fermerPinceWithTimeout),
                groundWithSecurityCommand,
                ouvrirPinceWithTimeout,
                remonterAscenseur
        );
    }

}
