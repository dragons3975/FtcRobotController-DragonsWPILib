package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoseTeamPropReculeFerme extends SequentialCommandGroup {

    public PoseTeamPropReculeFerme(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        PinceOuvreDroitCommand ouvreDroit = new PinceOuvreDroitCommand(pinceSubsystem);
        AvanceAutoCommand recule = new AvanceAutoCommand(driveSubsystem, -20, 0);
        PinceFermeCommand fermePince = new PinceFermeCommand(pinceSubsystem);
        BrasRotationPosCommand brasPourPasserAuDessus = new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kPositionLeverUnPeu);

        addCommands(
                new WaitCommand(0.5),
                ouvreDroit,
                new WaitCommand(0.5),
                recule.withTimeout(10),
                new WaitCommand(0.5),
                fermePince,
                new WaitCommand(0.5),
                brasPourPasserAuDessus
        );
    }

}