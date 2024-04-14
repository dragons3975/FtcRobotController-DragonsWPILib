package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserToile extends SequentialCommandGroup {

    public PoserToile(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasPosCommand pos1 = new BrasPosCommand(brasSubsystem, 400);
        PinceOuvreCommand ouvrePince = new PinceOuvreCommand(pinceSubsystem);
        ExtentionAutoCommand extention = new ExtentionAutoCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosToile);

        addCommands(
                pos1,
                extention,
                ouvrePince
        );
    }
}