package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TranslaterAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        ParallelRaceGroup tourne1 = new TourneAutoCommand(driveSubsystem, 10).withTimeout(1);
        AvanceAutoCommand reculer = new AvanceAutoCommand(driveSubsystem, -0.5, 20);
        ParallelRaceGroup tourne = new TourneAutoCommand(driveSubsystem, -90).withTimeout(3);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, 30);
        TranslaterAutoCommand translater = new TranslaterAutoCommand(driveSubsystem, -0.5, 8);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 0.5, 5);
        //poserToile = groupe de commande

        addCommands(
                //tourne1,
                reculer,
                //poserSol,
                tourne,
                avancer,
                translater,
                avancer2
                //poserToile
        );
    }

}