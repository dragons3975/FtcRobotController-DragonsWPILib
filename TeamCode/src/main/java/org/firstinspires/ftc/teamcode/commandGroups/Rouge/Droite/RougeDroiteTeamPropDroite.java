package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroitBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToileProcheAuto;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeDroiteTeamPropDroite extends SequentialCommandGroup {

    public RougeDroiteTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroitBleuEtRouge poserDroit = new PoserTeamPropDroitBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);

        addCommands(
                poserDroit
        );
    }

}