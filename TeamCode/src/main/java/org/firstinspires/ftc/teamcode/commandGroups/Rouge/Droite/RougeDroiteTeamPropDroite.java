package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeDroiteTeamPropDroite extends SequentialCommandGroup {

    public RougeDroiteTeamPropDroite(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        //ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, Constants.AutonomousConstants.kPos1AvanceGauche);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 0.5, Constants.AutonomousConstants.kApproche);
        //DetectPixelCommand detect = new DetectPixelCommand(pixelDetectionSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        BrasCommandPos1 bras = new BrasCommandPos1(brasSubsystem, 100);
        PinceAutoCommandOuvre ouvre = new PinceAutoCommandOuvre(pinceSubsystem);

        addCommands(
                //avancer,
                //tourne,
                //avancer2//,
                //bras,
                //ouvre
        );
    }

}