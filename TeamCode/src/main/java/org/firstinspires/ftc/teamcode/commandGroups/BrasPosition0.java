package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreBrasRotationCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreExtentionCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclineSolSecuriteCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BrasPosition0 extends SequentialCommandGroup {
    // Aussi utilisee pour la calibration initiale

    public BrasPosition0(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasRotationPosCommand hauteurSecurite = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurSecurite); //Seulement si deja calibre
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        PinceInclinaisonHautCommand pinceHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        CalibreExtentionCommand calibreExtention = new CalibreExtentionCommand(brasSubsystem);
        PinceInclinaisonBasCommand pinceBas = new PinceInclinaisonBasCommand(pinceSubsystem);
        CalibreBrasRotationCommand calibreBras = new CalibreBrasRotationCommand(brasSubsystem);
        BrasRotationPosCommand hauteurSolMin = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurSolMin);
        PinceInclineSolSecuriteCommand pinceInclineSecurite = new PinceInclineSolSecuriteCommand(pinceSubsystem);
        PinceOuvreCommand ouvre = new PinceOuvreCommand(pinceSubsystem);


        addCommands(
                hauteurSecurite,
                ferme,
                pinceHaut,
                calibreExtention,
                pinceBas,
                calibreBras,
                hauteurSolMin,
                pinceInclineSecurite,
                ouvre
        );
    }

}