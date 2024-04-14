package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropMilieu poserMilieu = new PoserTeamPropMilieu(brasSubsystem, pinceSubsystem, driveSubsystem);
        AvanceAutoCommand tasse = new AvanceAutoCommand(driveSubsystem, 0, -30);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 20, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -90);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 220, 0);
        TourneAutoCommand tourne2 = new TourneAutoCommand(driveSubsystem, 180);
        PoserToile poserToile = new PoserToile(brasSubsystem, pinceSubsystem);



        addCommands(
                poserMilieu/*,
                tasse,
                avancer2,
                tourne,
                avancer,
                poserToile*/
        );
    }

}