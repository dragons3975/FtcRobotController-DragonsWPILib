package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeDroitCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreDroitCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserTeamPropMilieuBleuEtRouge extends SequentialCommandGroup {

    public PoserTeamPropMilieuBleuEtRouge(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        PinceFermeCommand fermePince = new PinceFermeCommand(pinceSubsystem);
        AvanceAutoCommand avance = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        PinceOuvreDroitCommand ouvreDroit = new PinceOuvreDroitCommand(pinceSubsystem);
        AvanceAutoCommand recule = new AvanceAutoCommand(driveSubsystem, -20, 0);
        PinceFermeDroitCommand fermePinceDroit = new PinceFermeDroitCommand(pinceSubsystem);



        addCommands(
                avance.withTimeout(10),
                new WaitCommand(0.5),
                ouvreDroit,
                new WaitCommand(0.5),
                recule.withTimeout(10),
                new WaitCommand(0.5),
                fermePinceDroit
        );
    }

}