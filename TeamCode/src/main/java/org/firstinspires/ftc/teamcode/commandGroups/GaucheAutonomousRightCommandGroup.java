package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class GaucheAutonomousRightCommandGroup extends SequentialCommandGroup {

    public GaucheAutonomousRightCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {

        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
        GaucheStartingAutonomousCommandGroup gaucheStart = new GaucheStartingAutonomousCommandGroup(telemetry, driveSubsystem, ascenseurSubsystem, pinceSubsystem, gamepad);

        GoToAngleCommand TurnLeft30 = new GoToAngleCommand(telemetry, driveSubsystem,  30);
        GoToAngleCommand GoStraight = new GoToAngleCommand(telemetry, driveSubsystem,0);
        DriveAutoCommand reculer110cm = new DriveAutoCommand(telemetry, driveSubsystem, -110, 0);
        DriveAutoCommand replacer = new DriveAutoCommand(telemetry, driveSubsystem, -17, 5);

        addCommands(
                gaucheStart,
                reculer110cm.andThen(TurnLeft30).alongWith(goHigh),
                lacherCone,
                replacer,
                GoStraight
        );
    }

}
