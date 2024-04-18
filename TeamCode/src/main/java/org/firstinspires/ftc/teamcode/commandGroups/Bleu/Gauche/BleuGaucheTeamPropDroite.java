package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroitBleuEtRouge;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BleuGaucheTeamPropDroite extends SequentialCommandGroup {

    public BleuGaucheTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroitBleuEtRouge poserDroit = new PoserTeamPropDroitBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserDroit
        );
    }

}