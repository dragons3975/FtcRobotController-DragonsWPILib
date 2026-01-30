package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.AutonomousCommands.DriveAutonomusCommand;
import org.firstinspires.ftc.teamcode.AutonomousCommands.TurnAutonomousCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MainAutonomousCommand extends SequentialCommandGroup {

    public MainAutonomousCommand(DriveSubsystem driveSubsystem, RamasseurSubsystem ramasseurSubsystem, LanceurSubsystem lanceurSubsystem) {
        DriveAutonomusCommand recule = new DriveAutonomusCommand(driveSubsystem, 0, -100);
        LanceAutonomousCommand lance = new LanceAutonomousCommand(ramasseurSubsystem, lanceurSubsystem);
       // ParallelRaceGroup retourneVersApril = new TurnAutonomousCommand(driveSubsystem).withTimeout(0.2);
      //  DriveAutonomusCommand reculeBase = new DriveAutonomusCommand(driveSubsystem,0 , -100);

        addCommands(
                recule,
                lance
        );
    }

}
//ParallelRaceGroup pickUpBalle = new RamasseurCommand(ramasseurSubsystem).withTimeout(1.5);
//ParallelRaceGroup retourneVersBut = new TurnAutonomousCommand(driveSubsystem).withTimeout(0.3);
