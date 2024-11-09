package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroit;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileProcheAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PrendrePile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeDroiteTeamPropDroite extends SequentialCommandGroup {

    public RougeDroiteTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroit poserDroit = new PoserTeamPropDroit(brasSubsystem, pinceSubsystem, driveSubsystem);
        PrendrePile prendrePile = new PrendrePile(brasSubsystem, pinceSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 180);
        AvanceAutoCommand tasse = new AvanceAutoCommand(driveSubsystem, 0, -20);
        AvanceAutoCommand tasse2 = new AvanceAutoCommand(driveSubsystem, 0, -10);
        PoserToileProcheAuto poserToile = new PoserToileProcheAuto(brasSubsystem, pinceSubsystem, driveSubsystem);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);

        addCommands(
                poserDroit,
                new WaitCommand(0.5),
                tasse,
                new WaitCommand(0.5),
                tourne,
                poserToile,
                tasse2,
                ouvreGauche
        );
    }

}