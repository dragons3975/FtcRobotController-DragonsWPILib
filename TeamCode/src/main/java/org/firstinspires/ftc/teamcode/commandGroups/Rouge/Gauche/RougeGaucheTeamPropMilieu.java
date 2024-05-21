package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieuBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.AllerVersToileLoin;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropMilieuBleuEtRouge poserMilieu = new PoserTeamPropMilieuBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);
        TourneAutoCommand tourneGauche = new TourneAutoCommand(driveSubsystem, -90);
        TourneAutoCommand tourne180 = new TourneAutoCommand(driveSubsystem, 180);
        AvanceAutoCommand tasseDroit = new AvanceAutoCommand(driveSubsystem, 0, 7);
        AvanceAutoCommand tasseGauche = new AvanceAutoCommand(driveSubsystem, 0, -5);
        AvanceAutoCommand avance = new AvanceAutoCommand(driveSubsystem, 18, 0);
        AllerVersToileLoin allerVersToileLoin = new AllerVersToileLoin(brasSubsystem, pinceSubsystem, driveSubsystem);
        PinceOuvreCommand pinceOuvre = new PinceOuvreCommand(pinceSubsystem);
        PinceFermeCommand pinceFerme = new PinceFermeCommand(pinceSubsystem);
        AvanceAutoCommand tasseFin = new AvanceAutoCommand(driveSubsystem, 0, -Constants.AutonomousConstants.kTasseToileGarer);
        BrasRotationPosCommand brasPosPile = new BrasRotationPosCommand(brasSubsystem, 1500);
        BrasExtentionPosCommand extPile = new BrasExtentionPosCommand(brasSubsystem, 27200);
        BrasExtentionPosCommand ext0 = new BrasExtentionPosCommand(brasSubsystem, 0);

        addCommands(
                poserMilieu,
                tourneGauche,
                new WaitCommand(0.5),
                tasseDroit,
                avance,
                brasPosPile,
                pinceOuvre,
                extPile,
                new WaitCommand(0.5),
                pinceFerme,
                ext0,
                tourne180,
                tasseGauche,
                allerVersToileLoin

        );
    }
}