package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceFermeCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserTeamPropMilieuBleuEtRouge extends SequentialCommandGroup {

    public PoserTeamPropMilieuBleuEtRouge(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avance = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitialMilieu, 0);
        PoseTeamPropReculeFerme poseTeamPropReculeFerme = new PoseTeamPropReculeFerme(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                avance.withTimeout(10),
                poseTeamPropReculeFerme
        );
    }

}