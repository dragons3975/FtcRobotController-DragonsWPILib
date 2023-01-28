package org.firstinspires.ftc.teamcode.commandGroups;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.autonomous.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class GaucheAutonomousLeftCommandGroup extends SequentialCommandGroup {

    public GaucheAutonomousLeftCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {
        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
        GaucheStartingAutonomousCommandGroup gaucheStart = new GaucheStartingAutonomousCommandGroup(telemetry, driveSubsystem, ascenseurSubsystem, pinceSubsystem, gamepad);
        PlotsSequentialCommandGroup ramasserCone = new PlotsSequentialCommandGroup(telemetry, driveSubsystem, pinceSubsystem, ascenseurSubsystem);

        DriveAutoCommand reculer70cm = new DriveAutoCommand(telemetry, driveSubsystem, -Constants.Autonomous.k6_Y_2Case, 0);
        GoToAngleCommand turnLeft30 = new GoToAngleCommand(telemetry, driveSubsystem, -Constants.Autonomous.k7_Z_ForwardJonction);
        GoToAngleCommand turnLeft = new GoToAngleCommand(telemetry, driveSubsystem, 90);
        DriveAutoCommand avancer60cm = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k8_Y_2Case, 0);
        DriveAutoCommand avancer20cmSlow = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k9_Y_Slow, 0, Constants.Autonomous.kSlowSpeed);


        addCommands(
                gaucheStart,
                reculer70cm.andThen(turnLeft30).alongWith(goHigh),
                lacherCone,
                turnLeft,
                avancer60cm,
                avancer20cmSlow,
                ramasserCone
        );
    }

}
