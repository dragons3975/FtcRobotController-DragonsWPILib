package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeveBrasHauteurToileCommand extends SequentialCommandGroup {

    public LeveBrasHauteurToileCommand(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        BrasPosCommand bras = new BrasPosCommand(brasSubsystem, Constants.BrasConstants.kRotationPositionToile);
        ExtentionAutoCommand extention = new ExtentionAutoCommand(brasSubsystem, 500);
        PinceInclinaisonBasCommand pinceBas = new PinceInclinaisonBasCommand(pinceSubsystem);
        PinceInclinaisonHautCommand pinceHaut = new PinceInclinaisonHautCommand(pinceSubsystem);
        CalibreBrasCommand CalibreBras = new CalibreBrasCommand(brasSubsystem);

        addCommands(
                ferme,
                bras,
                pinceHaut,
                extention
                //poserToileb
        );
    }

}