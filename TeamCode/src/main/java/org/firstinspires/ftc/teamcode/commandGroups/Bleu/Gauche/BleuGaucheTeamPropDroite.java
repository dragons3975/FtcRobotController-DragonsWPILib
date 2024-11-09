package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroit;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileLoingAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileProcheAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PrendrePile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuGaucheTeamPropDroite extends SequentialCommandGroup {

    public BleuGaucheTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroit PoserDroit = new PoserTeamPropDroit(brasSubsystem, pinceSubsystem, driveSubsystem);
        PrendrePile prendrePile = new PrendrePile(brasSubsystem, pinceSubsystem);
        PoserToileProcheAuto posertOILE = new PoserToileProcheAuto(brasSubsystem, pinceSubsystem, driveSubsystem);
        AvanceAutoCommand tasse2 = new AvanceAutoCommand(driveSubsystem, 0, 20);
        PinceOuvreDroitCommand ouvreDroit = new PinceOuvreDroitCommand(pinceSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -180);

        addCommands(
                PoserDroit,
                new WaitCommand(0.5),
                tourne,
                new WaitCommand(0.5),
                posertOILE,
                tasse2,
                ouvreDroit
        );
    }

}