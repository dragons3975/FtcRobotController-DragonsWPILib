package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropMilieuBleuEtRouge;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeDroiteTeamPropMilieu extends SequentialCommandGroup {

    public RougeDroiteTeamPropMilieu(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropMilieuBleuEtRouge poserMilieu = new PoserTeamPropMilieuBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserMilieu,
                new AvanceAutoCommand(driveSubsystem, -77, 0).withTimeout(3),
                new AvanceAutoCommand(driveSubsystem, 0, 100).withTimeout(3)
        );
    }

}