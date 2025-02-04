package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceXAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceYAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Gauche extends SequentialCommandGroup {

    public Gauche(DriveSubsystem driveSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        AvanceXAutoCommand avance = new AvanceXAutoCommand(driveSubsystem,60);
        AvanceYAutoCommand tasse = new AvanceYAutoCommand(driveSubsystem, -60);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);

        addCommands(
                avance,
                new WaitCommand(0.2),
                tasse,
                new WaitCommand(0.2),
                tourne
        );
    }

}