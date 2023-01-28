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

public class DroiteAutonomousRightCommandGroup extends SequentialCommandGroup {

    public DroiteAutonomousRightCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem, PinceSubsystem pinceSubsystem, Gamepad gamepad) {

        AscenseurCommand goHigh = new AscenseurCommand(telemetry, ascenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);
        LacherConeSemiAutonomousCommandGroup lacherCone = new LacherConeSemiAutonomousCommandGroup( telemetry,  driveSubsystem,  pinceSubsystem,  ascenseurSubsystem);
        PlotsSequentialCommandGroup ramasserCone = new PlotsSequentialCommandGroup(telemetry, driveSubsystem, pinceSubsystem, ascenseurSubsystem);
        DroiteStartingAutonomousCommandGroup droiteStart = new DroiteStartingAutonomousCommandGroup(telemetry, driveSubsystem, ascenseurSubsystem, pinceSubsystem, gamepad);

        DriveAutoCommand reculer70cm = new DriveAutoCommand(telemetry, driveSubsystem, -Constants.Autonomous.k6_Y_2Case, 0);
        GoToAngleCommand turnRight30 = new GoToAngleCommand(telemetry, driveSubsystem, Constants.Autonomous.k7_Z_ForwardJonction);
        GoToAngleCommand turnRight = new GoToAngleCommand(telemetry, driveSubsystem, -90);
        DriveAutoCommand avancer60cm = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k8_Y_2Case, 0);
        DriveAutoCommand avancer20cmSlow = new DriveAutoCommand(telemetry, driveSubsystem, Constants.Autonomous.k9_Y_Slow, 0, Constants.Autonomous.kSlowSpeed);


        addCommands(
                droiteStart,
                reculer70cm.andThen(turnRight30).alongWith(goHigh),
                lacherCone,
                turnRight,
                avancer60cm,
                avancer20cmSlow,
                ramasserCone
        );
    }

}
