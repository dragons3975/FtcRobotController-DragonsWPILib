package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeDroiteTeamPropGauche extends SequentialCommandGroup {

    public RougeDroiteTeamPropGauche(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        //ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        AvanceAutoCommand reculer = new AvanceAutoCommand(driveSubsystem, -0.5, 15);
        AvanceAutoCommand reculer2 = new AvanceAutoCommand(driveSubsystem, -0.5, 3);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, 15);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 0.5, 6);
        AvanceAutoCommand avancer3 = new AvanceAutoCommand(driveSubsystem, 0.5, 2);
        TourneAutoCommand tourne2 = new TourneAutoCommand(driveSubsystem, 90);
        //DetectPixelCommand detect = new DetectPixelCommand(pixelDetectionSubsystem);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, -90);
        TourneAutoCommand tourne3 = new TourneAutoCommand(driveSubsystem, -90);
        BrasCommandPos1 bras = new BrasCommandPos1(brasSubsystem, 100);
        PinceAutoCommandOuvre ouvre = new PinceAutoCommandOuvre(pinceSubsystem);

        addCommands(
                //reculer,
                //poser
                //tourne,
                //reculer2,
                //ramasser
                //avancer,
                //tourne2,
                //avancer2,
                //tourne3,
                //avancer3
                //poser
                //bras,
                //ouvre
        );
    }

}