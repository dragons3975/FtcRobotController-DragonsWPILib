package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BrasCommandGroup extends SequentialCommandGroup {



    public BrasCommandGroup(BrasSubsystem brasSubsystem, int posMoteur, double posAvantBras, double posRotationMain) {

        BrasPosCommand command1 = new BrasPosCommand(brasSubsystem,posMoteur ,posAvantBras , -1);
        WaitCommand wait = new WaitCommand(3);
        BrasPosCommand command2 = new BrasPosCommand(brasSubsystem,-1  ,-1, posRotationMain);



        addCommands(
                command1,
                wait,
                command2
        );
    }
}