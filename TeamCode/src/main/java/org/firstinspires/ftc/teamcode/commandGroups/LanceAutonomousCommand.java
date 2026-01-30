package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RamasseurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RejetteCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.StopLanceurCommand;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LanceAutonomousCommand extends SequentialCommandGroup {

    public LanceAutonomousCommand(RamasseurSubsystem ramasseurSubsystem, LanceurSubsystem lanceurSubsystem) {

        //ParallelRaceGroup attrapeUnPeu = new RamasseurCommand(ramasseurSubsystem).withTimeout(0.2);
        ParallelRaceGroup rejetteUnPeu = new RejetteCommand(ramasseurSubsystem).withTimeout(0.2);
        LanceurCommand demarreLanceur = new LanceurCommand(lanceurSubsystem);
        ParallelRaceGroup pousse = new RamasseurCommand(ramasseurSubsystem).withTimeout(0.5);
        StopLanceurCommand stopLanceur = new StopLanceurCommand(lanceurSubsystem);

        addCommands(
                ///attrapeUnPeu,
                rejetteUnPeu,
                demarreLanceur,
                pousse,
                stopLanceur
        );
    }
}
