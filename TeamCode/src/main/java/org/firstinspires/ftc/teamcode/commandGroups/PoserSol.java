package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserSol extends SequentialCommandGroup {

    public PoserSol(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasRotationPosCommand bras = new BrasRotationPosCommand(brasSubsystem, 230);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosSol);
        PinceOuvreCommand ouvre = new PinceOuvreCommand(pinceSubsystem);

        addCommands(
                bras,
                extention,
                ouvre
                //poserToile
        );
    }

}