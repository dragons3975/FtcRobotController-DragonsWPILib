package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class StopLanceurCommand extends Command {

    private final LanceurSubsystem mLanceurSubsystem;

    public StopLanceurCommand(LanceurSubsystem LanceurSubsystem) {
        mLanceurSubsystem = LanceurSubsystem;
        addRequirements(LanceurSubsystem);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        mLanceurSubsystem.setDeltaMoyConsigne(0);
    }

}
