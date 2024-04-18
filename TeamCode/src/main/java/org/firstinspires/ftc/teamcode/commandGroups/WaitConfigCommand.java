package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreBrasRotationCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreExtentionCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonBasCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclinaisonHautCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceInclineSolSecuriteCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitConfigCommand extends SequentialCommandGroup {

    public WaitConfigCommand(BrasSubsystem brasSubsystem) {

        BrasRotationPosCommand hauteurCamera = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurCamera); //Seulement si deja calibre

        addCommands(
                hauteurCamera,
                new WaitCommand(4)
        );
    }

}