package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeGaucheCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonPile;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrendrePile extends SequentialCommandGroup {

    public PrendrePile(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand brasPositionPile = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kPositionPile);
        PinceOuvreDroitCommand ouvreDroit = new PinceOuvreDroitCommand(pinceSubsystem);
        PinceInclinaisonHautCommand inclineHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, Constants.BrasConstants.kExtentionMinimalToile);
        PinceInclinaisonPile inclinePile = new PinceInclinaisonPile(pinceSubsystem);
        BrasExtentionPosCommand extention2 = new BrasExtentionPosCommand(brasSubsystem, Constants.BrasConstants.kExtentionToile);
        PinceFermeGaucheCommand fermeGauche = new PinceFermeGaucheCommand(pinceSubsystem);

        addCommands(
                ferme,
                brasPositionPile,
                ouvreDroit,
                inclineHaut,
                extention,
                inclinePile,
                extention2,
                fermeGauche
        );
    }

}