package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.AutonomousCommands.DriveAutonomusCommand;
import org.firstinspires.ftc.teamcode.AutonomousCommands.TurnAutonomousCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RamasseurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RejetteCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MainAutonomousCommand extends SequentialCommandGroup {

    public MainAutonomousCommand(DriveSubsystem driveSubsystem, RamasseurSubsystem ramasseurSubsystem, LanceurSubsystem lanceurSubsystem) {

        ParallelRaceGroup attrapeUnPeu = new RamasseurCommand(ramasseurSubsystem).withTimeout(0.1);
        ParallelRaceGroup rejetteUnPeu = new RejetteCommand(ramasseurSubsystem).withTimeout(0.1);
        LanceurCommand demarreLanceur = new LanceurCommand(lanceurSubsystem);
        RamasseurCommand pousse = new RamasseurCommand(ramasseurSubsystem);


        addCommands(
                attrapeUnPeu,
                rejetteUnPeu,
                demarreLanceur,
                pousse
        );
    }

}
