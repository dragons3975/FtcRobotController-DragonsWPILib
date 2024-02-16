package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.PinceAutoOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggle;
import org.firstinspires.ftc.teamcode.commands.RamasseAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TasseAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PinceSubsystem pinceSubsystem) {

        //ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, 100);
        TasseAutoCommand tasser = new TasseAutoCommand(driveSubsystem, 0.5, 100);
        TourneAutoCommand tourner = new TourneAutoCommand(driveSubsystem, 30);
        ParallelRaceGroup ramasse = new RamasseAutoCommand(intakeSubsystem).withTimeout(2);
        PinceAutoOuvreCommand ouvre = new PinceAutoOuvreCommand(pinceSubsystem);
        addCommands(
                avancer,
                tasser,
                tourner,
                ramasse,
                ouvre
                //calibration
                );
    }

}