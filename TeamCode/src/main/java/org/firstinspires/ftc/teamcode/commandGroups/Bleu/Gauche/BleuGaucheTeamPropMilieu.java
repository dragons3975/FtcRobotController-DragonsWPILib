package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.LeveBrasHauteurToileCommand;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieuBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileLoingAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileProcheAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PrendrePile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuGaucheTeamPropMilieu extends SequentialCommandGroup {

    public BleuGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropMilieuBleuEtRouge PoserMilieu = new PoserTeamPropMilieuBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);
        PrendrePile prendrePile = new PrendrePile(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand reculer = new AvanceAutoCommand(driveSubsystem, -10, 0);
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -91);
        AvanceAutoCommand tasse = new AvanceAutoCommand(driveSubsystem, 0, -15);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne - 20, 0);
        LeveBrasHauteurToileCommand leve = new LeveBrasHauteurToileCommand(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 20, 0);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);
        PoserToileProcheAuto poserToile = new PoserToileProcheAuto(brasSubsystem, pinceSubsystem, driveSubsystem);


        addCommands(
                PoserMilieu,
                new WaitCommand(0.5),
                ferme,
                tourne,
                new WaitCommand(0.5),
                tasse,
                new WaitCommand(0.5),
                poserToile,
                ouvreGauche
        );
    }

}