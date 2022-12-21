package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousLeftCommandGroup extends SequentialCommandGroup {

    public AutonomousLeftCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, Gamepad gamepad) {

        DriveAutoCommand avancer40Cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        AscenseurCommand positionPickUp = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        AscenseurCommand positionBas = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        AscenseurCommand positionMoyen = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionMoyen);
        AscenseurCommand positionHaut = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);

        GoToAngleCommand GoStraight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 0);
        GoToAngleCommand GoLeft = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 90);
        GoToAngleCommand GoRight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, -90);
        GoToAngleCommand GoBack = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 180);

        DriveAutoCommand reculer40cm = new DriveAutoCommand(telemetry, driveSubsystem, -40, 0);
        DriveAutoCommand reculer10cm = new DriveAutoCommand(telemetry, driveSubsystem, -10, 0);



        addCommands(
                GoLeft,
                avancer40Cm,
                //ouvrir pince
                reculer10cm,
                GoRight,
                avancer40Cm
            );
    }

}
