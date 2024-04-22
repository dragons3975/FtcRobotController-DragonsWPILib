package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserTeamPropDroitBleuEtRouge extends SequentialCommandGroup {

    public PoserTeamPropDroitBleuEtRouge(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance  = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitialGaucheDroite, 0);
        TourneAutoCommand tourneDroite = new TourneAutoCommand(driveSubsystem, 90);
        AvanceAutoCommand avance2  = new AvanceAutoCommand(driveSubsystem, 10, 0);
        PoseTeamPropReculeFerme poseTeamPropReculeFerme = new PoseTeamPropReculeFerme(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                new BrasPosition0(brasSubsystem, pinceSubsystem),
                new PinceFermeCommand(pinceSubsystem),
                avance,
                tourneDroite,
                new WaitCommand(0.5),
                avance2,
                poseTeamPropReculeFerme
        );
    }

}
