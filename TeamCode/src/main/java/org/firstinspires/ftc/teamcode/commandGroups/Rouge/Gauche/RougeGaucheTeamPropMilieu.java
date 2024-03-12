package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheTeamPropMilieu extends SequentialCommandGroup {

    public RougeGaucheTeamPropMilieu(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand reculer = new AvanceAutoCommand(driveSubsystem, 0.5, 15);
        //poserSol = groupe de commande qui etend, ouvre, revient
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -90);
        AvanceAutoCommand reculer2 = new AvanceAutoCommand(driveSubsystem, -0.5, 3);
        //ramasser
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, 6);
        AvanceAutoCommand translater = new AvanceAutoCommand(driveSubsystem, 0.5, 6); //en y, pas en x
        //poserToile = groupe de commande

        addCommands(
                reculer//,
                //poserSol
                //tourne,
                //reculer2,
                //ramasser
                //avancer,
                //translater//,
                //poserToile
        );
    }

}