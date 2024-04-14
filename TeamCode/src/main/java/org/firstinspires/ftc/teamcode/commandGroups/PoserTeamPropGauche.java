package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserTeamPropGauche extends SequentialCommandGroup {

    public PoserTeamPropGauche(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance  = new AvanceAutoCommand(driveSubsystem, 80, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -90);
        AvanceAutoCommand avance2  = new AvanceAutoCommand(driveSubsystem, 5, 0);
        PinceOuvreDroitCommand poser = new PinceOuvreDroitCommand(pinceSubsystem);
        AvanceAutoCommand recule  = new AvanceAutoCommand(driveSubsystem, -20, 0);
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand bras = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kPositionLeverUnPeu);
        AvanceAutoCommand avance3  = new AvanceAutoCommand(driveSubsystem, 40, 0);

        addCommands(
                avance,
                tourne,
                new WaitCommand(0.5),
                avance2,
                poser,
                new WaitCommand(0.5),
                recule,
                new WaitCommand(0.5),
                ferme,
                new WaitCommand(0.5),
                bras
        );
    }

}