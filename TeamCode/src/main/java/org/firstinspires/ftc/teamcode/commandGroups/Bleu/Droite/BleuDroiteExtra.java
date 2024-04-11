package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BleuDroiteExtra extends SequentialCommandGroup {

    public BleuDroiteExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne, 0);
        AvanceAutoCommand avancer3 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementExtra2, 0);
        AvanceAutoCommand tasser = new AvanceAutoCommand(driveSubsystem, 0, -Constants.AutonomousConstants.kDeplacementExtraLateral);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        TourneAutoCommand tourne2 = new TourneAutoCommand(driveSubsystem, -90);
        CalibreBrasCommand calibre = new CalibreBrasCommand(brasSubsystem);
        BrasCommandPos pos1 = new BrasCommandPos(brasSubsystem, 400);
        PinceAutoCommandOuvre ouvrePince = new PinceAutoCommandOuvre(pinceSubsystem);



        addCommands(
                //calibre,
                avancer,
                tourne,
                avancer2,
                tourne2,
                tasser,
                avancer3,
                pos1,
                ouvrePince
        );
    }

}