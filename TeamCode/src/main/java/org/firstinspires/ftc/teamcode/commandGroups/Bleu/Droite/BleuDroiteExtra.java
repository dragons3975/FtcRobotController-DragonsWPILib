package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.PinceOpenCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuDroiteExtra extends SequentialCommandGroup {

    public BleuDroiteExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, XboxController xboxController) {
//        pas de pid pour le moment donc avec un timeout
        ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0).withTimeout(0.3);
        ParallelRaceGroup tasserdroite = new AvanceAutoCommand(driveSubsystem, 0, 1).withTimeout(0.3);
        ParallelRaceGroup avancer = new AvanceAutoCommand(driveSubsystem, 1, 0).withTimeout(0.3);
        ParallelRaceGroup tassergauche = new AvanceAutoCommand(driveSubsystem, 0, -1).withTimeout(0.3);
        PinceOpenCommand pinceopen = new PinceOpenCommand(pinceSubsystem, xboxController);

        addCommands(
                reculer,
                new WaitCommand(0.2),
                tasserdroite,
                new WaitCommand(0.2),
                avancer,
                new WaitCommand(0.2),
                tassergauche,
                new WaitCommand(0.2),
                pinceopen

        );
    }

}