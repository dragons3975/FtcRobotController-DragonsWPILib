package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PrendrePile extends SequentialCommandGroup {

    public PrendrePile(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasPosCommand bras = new BrasPosCommand(brasSubsystem, 300);
        ExtentionAutoCommand extention = new ExtentionAutoCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosSol);
        PinceFermeGaucheCommand fermeGauche = new PinceFermeGaucheCommand(pinceSubsystem);

        addCommands(
                bras,
                extention,
                fermeGauche
                //poserToile
        );
    }

}