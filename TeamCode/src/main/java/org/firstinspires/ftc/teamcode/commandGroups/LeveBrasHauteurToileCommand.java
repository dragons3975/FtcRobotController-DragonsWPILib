package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreBrasRotationCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeveBrasHauteurToileCommand extends SequentialCommandGroup {

    public LeveBrasHauteurToileCommand(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand bras = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kRotationPositionToile);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, 1000);
        PinceInclinaisonBasCommand pinceBas = new PinceInclinaisonBasCommand(pinceSubsystem);
        PinceInclinaisonHautCommand pinceHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        CalibreBrasRotationCommand CalibreBras = new CalibreBrasRotationCommand(brasSubsystem);

        addCommands(
                ferme,
                bras,
                pinceHaut,
                extention
                //poserToileb
        );
    }

}