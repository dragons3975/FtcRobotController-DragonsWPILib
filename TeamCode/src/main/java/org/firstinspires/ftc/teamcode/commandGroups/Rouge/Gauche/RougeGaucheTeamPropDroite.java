package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropDroitBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.AllerVersToileLoin;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeGaucheTeamPropDroite extends SequentialCommandGroup {

    public RougeGaucheTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropDroitBleuEtRouge poserDroit = new PoserTeamPropDroitBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);
        AllerVersToileLoin allerVersToileLoin = new AllerVersToileLoin(brasSubsystem, pinceSubsystem, driveSubsystem);
        AvanceAutoCommand tasseDroite = new AvanceAutoCommand(driveSubsystem, 0, Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag);
        PinceOuvreCommand pinceOuvre = new PinceOuvreCommand(pinceSubsystem);
        AvanceAutoCommand tasseFin = new AvanceAutoCommand(driveSubsystem, 0, -(Constants.AutonomousConstants.kTasseToileGarer + Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag));

        addCommands(
                poserDroit,
                new WaitCommand(0.5),
                allerVersToileLoin,
                tasseDroite,
                pinceOuvre,
                tasseFin
        );
    }
}
