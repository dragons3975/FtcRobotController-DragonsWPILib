package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.ParallelCommandGroup;
import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class DroiteStartingAutonomousCommandGroup extends SequentialCommandGroup {

    public DroiteStartingAutonomousCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {
        ParallelRaceGroup ouvrirPince = new OuvrirPinceCommand(telemetry, pinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);
        AscenseurCommand goLow = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
        PlotsSequentialCommandGroup attrapeCone = new PlotsSequentialCommandGroup(telemetry, driveSubsystem, pinceSubsystem, ascenseurSubsystem);

        DriveAutoCommand hoverLeft = new DriveAutoCommand(telemetry, driveSubsystem, 0, -Constants.Autonomous.k1_Y_1Case);
        DriveAutoCommand avancerCentral = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k2_X_2Case, 0);
        GoToAngleCommand turnLeftDown = new GoToAngleCommand(telemetry, driveSubsystem, Constants.Autonomous.k3_Z_CentralJonction);
        GoToAngleCommand turnRight = new GoToAngleCommand(telemetry, driveSubsystem, -90);
        DriveAutoCommand avancerPilePlots = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k4_Y_2Case, 0);
        DriveAutoCommand avancerSlow = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k5_Y_Slow, 0, 0.5);

        addCommands(
                ouvrirPince,
                (hoverLeft.andThen(avancerCentral)).alongWith(goHigh),
                turnLeftDown,
                lacherCone,
                turnRight.andThen(avancerPilePlots).andThen(avancerSlow).alongWith(goLow),
                attrapeCone
        );

//        AscenseurCommand goLow = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
//        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
//        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
//        PlotsSequentialCommandGroup attrapeCone = new PlotsSequentialCommandGroup(telemetry, driveSubsystem, pinceSubsystem, ascenseurSubsystem);
//
//
//        GoToAngleCommand TurnLeft135 = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, 135);
//        GoToAngleCommand GoLeft = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, 90);
//        GoToAngleCommand GoRight = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, -90);
//
//        DriveAutoCommand HoverLeft = new DriveAutoCommand(telemetry, driveSubsystem, 0, -60);
//        DriveAutoCommand avancerUp = new DriveAutoCommand(telemetry, driveSubsystem, 122, 0);
//        DriveAutoCommand avancerPile = new DriveAutoCommand(telemetry, driveSubsystem, 123, 0);
//        DriveAutoCommand avancer7CmSlow = new DriveAutoCommand(telemetry, driveSubsystem, 7, 0, 0.5);
//
//        addCommands(
//                (HoverLeft.andThen(avancerUp)).alongWith(goHigh),
//                TurnLeft135,
//                lacherConeSemiAutonomousCommandGroup,
//                GoRight.andThen(avancerPile).andThen(avancer7CmSlow).alongWith(goLow),
//                plotsSequentialCommandGroup
//        );
    }

}
