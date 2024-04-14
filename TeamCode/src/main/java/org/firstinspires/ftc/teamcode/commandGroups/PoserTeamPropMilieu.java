package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserTeamPropMilieu extends SequentialCommandGroup {

    public PoserTeamPropMilieu(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance  = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        PoserSol1Seul poser = new PoserSol1Seul(brasSubsystem, pinceSubsystem);


        addCommands(
                avance,
                poser
        );
    }

}