package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeveBrasHauteurToileBasCommand extends SequentialCommandGroup {

    public LeveBrasHauteurToileBasCommand(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        PinceFermeCommand fermePince = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand brasRotationPositionToile = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kRotationPositionToileBas);
        PinceInclinaisonHautCommand pinceHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, Constants.BrasConstants.kExtentionMinimalToile);
        PinceInclinaisonBasCommand pinceBas = new PinceInclinaisonBasCommand(pinceSubsystem);
        BrasExtentionPosCommand extentionFinal = new BrasExtentionPosCommand(brasSubsystem, Constants.BrasConstants.kExtentionToileBas);

        addCommands(
                fermePince,
                brasRotationPositionToile,
                pinceHaut,
                extention,
                pinceBas,
                extentionFinal
                //poserToileb
        );
    }

}