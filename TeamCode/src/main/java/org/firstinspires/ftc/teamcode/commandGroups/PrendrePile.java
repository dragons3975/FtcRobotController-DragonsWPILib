package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrendrePile extends SequentialCommandGroup {

    public PrendrePile(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasRotationPosCommand bras = new BrasRotationPosCommand(brasSubsystem, 300);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosSol);
        PinceFermeGaucheCommand fermeGauche = new PinceFermeGaucheCommand(pinceSubsystem);

        addCommands(
                bras,
                extention,
                fermeGauche
                //poserToile
        );
    }

}