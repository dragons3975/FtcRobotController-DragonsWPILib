package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserSol;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheExtra extends SequentialCommandGroup {

    public RougeGaucheExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne, 0);
        AvanceAutoCommand avancer3 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementExtra2, 0);
        AvanceAutoCommand tasser = new AvanceAutoCommand(driveSubsystem, 0, -Constants.AutonomousConstants.kDeplacementExtraLateral);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        TourneAutoCommand tourne2 = new TourneAutoCommand(driveSubsystem, -90);
        BrasPosCommand pos1 = new BrasPosCommand(brasSubsystem, 400);
        PinceOuvreCommand ouvrePince = new PinceOuvreCommand(pinceSubsystem);
        PoserSol poserSol = new PoserSol(brasSubsystem, pinceSubsystem);



        addCommands(
                avancer,
                tourne,
                avancer2,
                tourne2,
                tasser,
                avancer3,
                poserSol
        );
    }

}