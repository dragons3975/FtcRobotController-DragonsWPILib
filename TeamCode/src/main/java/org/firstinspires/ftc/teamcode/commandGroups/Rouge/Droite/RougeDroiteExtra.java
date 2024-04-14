package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserSol;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeDroiteExtra extends SequentialCommandGroup {

    public RougeDroiteExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial + Constants.AutonomousConstants.kDeplacementExtraLateral, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementExtra2, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        BrasPosCommand pos1 = new BrasPosCommand(brasSubsystem, 400);
        PinceOuvreCommand ouvrePince = new PinceOuvreCommand(pinceSubsystem);
        ExtentionAutoCommand extention = new ExtentionAutoCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosToile);
        PoserSol poserSol = new PoserSol(brasSubsystem, pinceSubsystem);


        addCommands(
                avancer,
                tourne,
                avancer2,
                poserSol
        );
    }

}