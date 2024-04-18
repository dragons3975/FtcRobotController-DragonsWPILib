package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGaucheBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileProcheAuto;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuGaucheTeamPropGauche extends SequentialCommandGroup {

    public BleuGaucheTeamPropGauche(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropGaucheBleuEtRouge poserGauche = new PoserTeamPropGaucheBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserGauche
        );
    }

}