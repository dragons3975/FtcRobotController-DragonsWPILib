package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Gauche extends SequentialCommandGroup {

    public Gauche(DriveSubsystem driveSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);

        addCommands(
                //reculer
        );
    }

}