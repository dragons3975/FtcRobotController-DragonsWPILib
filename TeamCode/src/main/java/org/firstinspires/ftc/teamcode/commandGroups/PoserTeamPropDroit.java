package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserTeamPropDroit extends SequentialCommandGroup {

    public PoserTeamPropDroit(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance  = new AvanceAutoCommand(driveSubsystem, 10, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 10);
        AvanceAutoCommand avance2  = new AvanceAutoCommand(driveSubsystem, 40, 0);
        PoserSol1Seul poser = new PoserSol1Seul(brasSubsystem, pinceSubsystem);

        addCommands(
                avance,
                tourne,
                avance2,
                poser
        );
    }

}