package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreExtentionCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclineSolSecuriteCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BrasPosition0 extends SequentialCommandGroup {
    // Aussi utilisee pour la calibration initiale

    public BrasPosition0(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasPosCommand hauteurSecurite = new BrasPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurSecurite); //Seulement si deja calibre
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        PinceInclinaisonHautCommand pinceHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        CalibreExtentionCommand calibreExtention = new CalibreExtentionCommand(brasSubsystem);
        PinceInclinaisonBasCommand pinceBas = new PinceInclinaisonBasCommand(pinceSubsystem);
        CalibreBrasCommand calibreBras = new CalibreBrasCommand(brasSubsystem);
        BrasPosCommand hauteurSolMin = new BrasPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurSolMin);
        PinceInclineSolSecuriteCommand pinceInclineSecurite = new PinceInclineSolSecuriteCommand(pinceSubsystem);
        //Ouvrir les pinces


        addCommands(
                hauteurSecurite,
                ferme,
                pinceHaut,
                calibreExtention,
                pinceBas,
                calibreBras,
                hauteurSolMin,
                pinceInclineSecurite
        );
    }

}