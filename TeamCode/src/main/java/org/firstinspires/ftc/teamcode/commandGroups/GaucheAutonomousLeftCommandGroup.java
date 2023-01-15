package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class GaucheAutonomousLeftCommandGroup extends SequentialCommandGroup {

    public GaucheAutonomousLeftCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {

        DriveAutoCommand avancer130Cm = new DriveAutoCommand(telemetry, driveSubsystem, 130, 0);
        DriveAutoCommand reculer130cm = new DriveAutoCommand(telemetry, driveSubsystem, -130, 0);
        DriveAutoCommand avancer60cm = new DriveAutoCommand(telemetry, driveSubsystem, 60, 0);

        AscenseurCommand positionPickUp = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        AscenseurCommand positionBas = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        AscenseurCommand positionMoyen = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionMoyen);
        AscenseurCommand positionHaut = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);

        GoToAngleCommand GoStraight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 0);
        GoToAngleCommand GoLeft = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 90);
        GoToAngleCommand GoRight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, -90);
        GoToAngleCommand GoBack = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 180);
        GoToAngleCommand TurnRight45 = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, -45);
        GoToAngleCommand TurnLeft45 = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, 45);

        //x =  forward-backward
        //y = sideways

        DriveAutoCommand tourner20cm = new DriveAutoCommand(telemetry, driveSubsystem, 0, 20);

        DriveAutoCommand avancer40cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        DriveAutoCommand reculer30cm = new DriveAutoCommand(telemetry, driveSubsystem, -30, 0);

        DriveAutoCommand reculer10cm = new DriveAutoCommand(telemetry, driveSubsystem, -10, 0);
        OuvrirPinceCommand ouvrirPince = new OuvrirPinceCommand(telemetry, pinceSubsystem);
        FermerPinceCommand fermerPince = new FermerPinceCommand(telemetry, pinceSubsystem);


        addCommands(
                avancer130Cm,
                TurnRight45,
                avancer40cm,
                reculer10cm,
                //séquence ascenseur
                ouvrirPince,
                ouvrirPince,
                reculer30cm,
                GoStraight,
                reculer130cm,
                GoLeft,
                avancer60cm,
                GoStraight
            );
    }

}
