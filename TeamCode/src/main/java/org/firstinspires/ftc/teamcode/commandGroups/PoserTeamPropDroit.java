package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserTeamPropDroit extends SequentialCommandGroup {

    public PoserTeamPropDroit(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance  = new AvanceAutoCommand(driveSubsystem, 70, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        AvanceAutoCommand avance2  = new AvanceAutoCommand(driveSubsystem, 20, 0);
        PoserSol1Seul poser = new PoserSol1Seul(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand reculer  = new AvanceAutoCommand(driveSubsystem, -20, 0);
        PinceFermeCommand ferme = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand bras = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kPositionLeverUnPeu);

        addCommands(
                avance,
                tourne,
                avance2,
                poser,
                new WaitCommand(0.5),
                reculer,
                ferme,
                bras

        );
    }

}