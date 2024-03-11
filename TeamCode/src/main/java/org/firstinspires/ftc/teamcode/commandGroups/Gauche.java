package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelDetectionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Gauche extends SequentialCommandGroup {

    public Gauche(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PinceSubsystem pinceSubsystem, PixelDetectionSubsystem pixelDetectionSubsystem) {

        //ParallelRaceGroup calibration = new CalibreBrasCommand(brasSubsystem).withTimeout(0);
        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, 0.5, 100);
        //DetectPixelCommand detect = new DetectPixelCommand(pixelDetectionSubsystem);

        addCommands(
                avancer
                //detect
        );
    }

}