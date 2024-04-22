package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGaucheBleuEtRouge;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeDroiteTeamPropGauche extends SequentialCommandGroup {

    public RougeDroiteTeamPropGauche(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropGaucheBleuEtRouge poserGauche = new PoserTeamPropGaucheBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserGauche,
                new TourneAutoCommand(driveSubsystem, 90),
                new AvanceAutoCommand(driveSubsystem, -77, 0).withTimeout(3),
                new AvanceAutoCommand(driveSubsystem, 0, 100).withTimeout(3)
        );
    }

}