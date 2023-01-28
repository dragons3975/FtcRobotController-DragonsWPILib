package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.dragonswpilib.command.WaitCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class LacherConeSemiAutonomousCommandGroup extends SequentialCommandGroup {

    public LacherConeSemiAutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, AscenseurSubsystem ascenseurSubsystem) {

        DriveAutoCommand avancer = new DriveAutoCommand(telemetry, driveSubsystem, 43, 0, 0.7);
        DriveAutoCommand reculer = new DriveAutoCommand(telemetry, driveSubsystem, -10.5, 0);
        SequentialCommandGroup reculer11cmWithTimeout = new DriveAutoCommand(telemetry, driveSubsystem, -11.5, 0, 0.5).andThen(new WaitCommand(Constants.DriveConstants.kStillTimeout));

        ParallelRaceGroup fermerPinceWithTimeout = new FermerPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kFermerPinceTimeout);

        addCommands(
                avancer,
                reculer11cmWithTimeout,
                fermerPinceWithTimeout,
                reculer
        );
    }

}
