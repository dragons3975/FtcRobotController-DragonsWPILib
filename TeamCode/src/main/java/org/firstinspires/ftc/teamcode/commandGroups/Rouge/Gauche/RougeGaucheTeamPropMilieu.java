package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TranslaterAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileProche, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        CalibreBrasCommand calibre = new CalibreBrasCommand(brasSubsystem);
        BrasCommandPos pos1 = new BrasCommandPos(brasSubsystem, 400);
        PinceAutoCommandOuvre ouvrePince = new PinceAutoCommandOuvre(pinceSubsystem);



        addCommands(
                //calibre,
                avancer,
                tourne,
                avancer2,
                pos1,
                ouvrePince
        );
    }

}