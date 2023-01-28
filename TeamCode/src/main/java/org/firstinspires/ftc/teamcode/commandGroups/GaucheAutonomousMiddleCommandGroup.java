package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class GaucheAutonomousMiddleCommandGroup extends SequentialCommandGroup {

    public GaucheAutonomousMiddleCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {

        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
        GaucheStartingAutonomousCommandGroup gaucheStart = new GaucheStartingAutonomousCommandGroup(telemetry, driveSubsystem, ascenseurSubsystem, pinceSubsystem, gamepad);

        GoToAngleCommand turnLeft30 = new GoToAngleCommand(telemetry, driveSubsystem, -Constants.Autonomous.k7_Z_ForwardJonction);
        DriveAutoCommand reculer70cm = new DriveAutoCommand(telemetry, driveSubsystem, -Constants.Autonomous.k6_Y_2Case, 0);
        DriveAutoCommand replacer = new DriveAutoCommand(telemetry, driveSubsystem,
                Constants.Autonomous.k10_X_finalPos,
                -Constants.Autonomous.k10_Y_finalPos);
        GoToAngleCommand turnUp = new GoToAngleCommand(telemetry, driveSubsystem, 0);

        addCommands(
                gaucheStart,
                reculer70cm.andThen(turnLeft30).alongWith(goHigh),
                lacherCone,
                replacer,
                turnUp
        );
    }

}
