package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceXAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceYAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Droit extends SequentialCommandGroup {

    public Droit(DriveSubsystem driveSubsystem, BrasSubsystem brasSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        AvanceXAutoCommand avance = new AvanceXAutoCommand(driveSubsystem,60, 0);
        AvanceYAutoCommand tasse = new AvanceYAutoCommand(driveSubsystem, 60);
        AvanceXAutoCommand avance1 = new AvanceXAutoCommand(driveSubsystem,10, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        BrasPositionCommandtest panier = new BrasPositionCommandtest(brasSubsystem, -500, 0.5);

        addCommands(
                avance,
                new WaitCommand(0.2),
                panier
//                tasse,
//                new WaitCommand(0.2),
//                avance1,
//                new WaitCommand(0.2),
//                tourne,
//                new WaitCommand(0.2),
//                avance1,
//                new WaitCommand(0.2),
//                tourne,
//                new WaitCommand(0.2),
//                avance

        );
    }

}