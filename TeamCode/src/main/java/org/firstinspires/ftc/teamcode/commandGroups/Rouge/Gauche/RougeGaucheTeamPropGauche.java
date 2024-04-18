package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserTeamPropGaucheBleuEtRouge;
import org.firstinspires.ftc.teamcode.commandGroups.AllerVersToileLoin;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RougeGaucheTeamPropGauche extends SequentialCommandGroup {

    public RougeGaucheTeamPropGauche(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        PoserTeamPropGaucheBleuEtRouge poserGauche = new PoserTeamPropGaucheBleuEtRouge(brasSubsystem, pinceSubsystem, driveSubsystem);
        TourneAutoCommand demiTour = new TourneAutoCommand(driveSubsystem, 180);
        AllerVersToileLoin allerVersToileLoin = new AllerVersToileLoin(brasSubsystem, pinceSubsystem, driveSubsystem);
        AvanceAutoCommand tasseGauche = new AvanceAutoCommand(driveSubsystem, 0, -Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag);
        PinceOuvreCommand pinceOuvre = new PinceOuvreCommand(pinceSubsystem);
        AvanceAutoCommand tasseFin = new AvanceAutoCommand(driveSubsystem, 0, -(Constants.AutonomousConstants.kTasseToileGarer - Constants.AutonomousConstants.kTasseToileGaucheDroiteAprilTag));

        addCommands(
                poserGauche,
                new WaitCommand(0.5),
                demiTour,
                new WaitCommand(0.5),
                allerVersToileLoin,
                tasseGauche,
                pinceOuvre,
                tasseFin
        );
    }
}
