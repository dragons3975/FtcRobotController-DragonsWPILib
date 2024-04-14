package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToile;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileLoingAuto;
import org.firstinspires.ftc.teamcode.commandGroups.PrendrePile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeGaucheTeamPropGauche extends SequentialCommandGroup {

    public RougeGaucheTeamPropGauche(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropGauche poserGauche = new PoserTeamPropGauche(brasSubsystem, pinceSubsystem, driveSubsystem);
        PrendrePile prendrePile = new PrendrePile(brasSubsystem, pinceSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 180);
        AvanceAutoCommand tasse = new AvanceAutoCommand(driveSubsystem, 0, -20);
        AvanceAutoCommand tasse2 = new AvanceAutoCommand(driveSubsystem, 0, -10);
        PoserToileLoingAuto poserToile = new PoserToileLoingAuto(brasSubsystem, pinceSubsystem, driveSubsystem);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);

        addCommands(
                poserGauche,
                new WaitCommand(0.5),
                tasse,
                new WaitCommand(0.5),
                tourne,
                poserToile,
                tasse2,
                ouvreGauche/*
                avancer2,
                tourne,
                avancer,
                poserToile*/
        );
    }

}