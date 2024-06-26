package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserSol;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BleuGaucheExtra extends SequentialCommandGroup {

    public BleuGaucheExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial + Constants.AutonomousConstants.kDeplacementExtraLateral, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementExtra2, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -90);
        BrasRotationPosCommand pos1 = new BrasRotationPosCommand(brasSubsystem, 400);
        PinceOuvreCommand ouvrePince = new PinceOuvreCommand(pinceSubsystem);
        BrasExtentionPosCommand extention = new BrasExtentionPosCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosToile);
        PoserSol poserSol = new PoserSol(brasSubsystem, pinceSubsystem);


        addCommands(
                avancer,
                tourne,
                avancer2,
                poserSol
        );
    }

}