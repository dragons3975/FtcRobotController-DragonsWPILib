package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroitBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.AllerVersToileLoin;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuDroiteTeamPropDroite extends SequentialCommandGroup {

    public BleuDroiteTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroitBleuEtRouge poserDroit = new PoserTeamPropDroitBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);
        TourneAutoCommand demiTour = new TourneAutoCommand(driveSubsystem, 180);
        AllerVersToileLoin allerVersToileLoin = new AllerVersToileLoin(brasSubsystem, pinceSubsystem, driveSubsystem);
        AvanceAutoCommand tasseDroite = new AvanceAutoCommand(driveSubsystem, 0, Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag);
        PinceOuvreCommand pinceOuvre = new PinceOuvreCommand(pinceSubsystem);
        AvanceAutoCommand tasseFin = new AvanceAutoCommand(driveSubsystem, 0, Constants.AutonomousConstants.kTasseToileGarer - Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag);

        addCommands(
                poserDroit,
                new WaitCommand(0.5),
                demiTour,
                allerVersToileLoin,
                tasseDroite,
                pinceOuvre,
                tasseFin
        );
    }

}