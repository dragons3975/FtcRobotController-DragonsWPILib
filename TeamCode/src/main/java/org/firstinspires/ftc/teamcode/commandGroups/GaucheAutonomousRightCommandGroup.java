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
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class GaucheAutonomousRightCommandGroup extends SequentialCommandGroup {

    public GaucheAutonomousRightCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {

        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        DriveAutoCommand avancer130Cm = new DriveAutoCommand(telemetry, driveSubsystem, 130, 0);
        GoToAngleCommand TurnRight45 = new GoToAngleCommand(telemetry, driveSubsystem, gamepad, -45);
        LacherConeSemiAutonomousCommandGroup lacherConeSemiAutonomousCommandGroup = new LacherConeSemiAutonomousCommandGroup(telemetry, driveSubsystem, pinceSubsystem);
        GoToAngleCommand GoStraight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, 0);
        DriveAutoCommand reculer130cm = new DriveAutoCommand(telemetry, driveSubsystem, -130, 0);
        GoToAngleCommand GoRight = new GoToAngleCommand(telemetry, driveSubsystem,gamepad, -90);
        DriveAutoCommand avancer60cm = new DriveAutoCommand(telemetry, driveSubsystem, 60, 0);

        addCommands(
                goHigh,
                avancer130Cm,
                TurnRight45,
                lacherConeSemiAutonomousCommandGroup,
                GoStraight,
                reculer130cm,
                GoRight,
                avancer60cm,
                GoStraight
            );
    }

}
