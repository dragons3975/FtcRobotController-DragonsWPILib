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

public class ConeSequentialCommandGroup extends SequentialCommandGroup {

    public ConeSequentialCommandGroup(Telemetry telemetry, DriveSubsystem driveSubsystem, AscenseurSubsystem ascenseurSubsystem) {

        DriveAutoCommand avancer40Cm = new DriveAutoCommand(telemetry, driveSubsystem, 40, 0);
        DriveAutoCommand reculer10cm = new DriveAutoCommand(telemetry, driveSubsystem, -10, 0);




        addCommands(
                avancer40Cm,
                reculer10cm
            );
    }

}
