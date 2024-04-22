package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroitBleuEtRouge;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BleuGaucheTeamPropDroite extends SequentialCommandGroup {

    public BleuGaucheTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroitBleuEtRouge poserDroit = new PoserTeamPropDroitBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserDroit,
                new TourneAutoCommand(driveSubsystem, -90),
                new AvanceAutoCommand(driveSubsystem, -77, 0).withTimeout(3),
                new AvanceAutoCommand(driveSubsystem, 0, -100).withTimeout(3)
        );
    }

}