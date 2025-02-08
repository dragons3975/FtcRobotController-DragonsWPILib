package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceXAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceYAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Droit extends SequentialCommandGroup {

    public Droit(DriveSubsystem driveSubsystem, BrasSubsystem brasSubsystem) {
        ParallelRaceGroup avance = new AvanceXAutoCommand(driveSubsystem,90, 40).withTimeout(2);
        ParallelRaceGroup tasse = new AvanceXAutoCommand(driveSubsystem,90, 60).withTimeout(1);
        ParallelRaceGroup recule = new AvanceXAutoCommand(driveSubsystem,10, 60).withTimeout(2);

        ParallelRaceGroup avance2 = new AvanceXAutoCommand(driveSubsystem,90, 60).withTimeout(2);
        ParallelRaceGroup tasse2 = new AvanceXAutoCommand(driveSubsystem,90, 80).withTimeout(1);
        ParallelRaceGroup recule2 = new AvanceXAutoCommand(driveSubsystem,10, 80).withTimeout(2);

        ParallelRaceGroup avance3 = new AvanceXAutoCommand(driveSubsystem,90, 80).withTimeout(2);
        ParallelRaceGroup tasse3 = new AvanceXAutoCommand(driveSubsystem,90, 100).withTimeout(1);
        ParallelRaceGroup recule3 = new AvanceXAutoCommand(driveSubsystem,10, 100).withTimeout(2);


        addCommands(
                avance,
                tasse,
                recule,
                avance2,
                tasse2,
                recule2,
                avance3,
                tasse3,
                recule3
        );
    }

}