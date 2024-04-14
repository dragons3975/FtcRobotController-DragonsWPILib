package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.LeveBrasHauteurToileCommand;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroit;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToile;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileLoingAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PrendrePile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TasseAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropMilieu PoserMilieu = new PoserTeamPropMilieu(brasSubsystem, pinceSubsystem, driveSubsystem);
        PrendrePile prendrePile = new PrendrePile(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand reculer = new AvanceAutoCommand(driveSubsystem, -10, 0);
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 91);
        AvanceAutoCommand tasse = new AvanceAutoCommand(driveSubsystem, 0, -15);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne - 20, 0);
        LeveBrasHauteurToileCommand leve = new LeveBrasHauteurToileCommand(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 20, 0);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);
        PoserToileLoingAuto poserToile = new PoserToileLoingAuto(brasSubsystem, pinceSubsystem, driveSubsystem);


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
                //prendrePile,
                /*avancer2,
                tourne,
                avancer,
                poserToile*/
        );
    }

}