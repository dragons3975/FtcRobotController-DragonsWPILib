package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LanceurCommand extends Command {

    private final LanceurSubsystem mLanceurSubsystem;

    public LanceurCommand(LanceurSubsystem LanceurSubsystem) {
        mLanceurSubsystem = LanceurSubsystem;
        addRequirements(LanceurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mLanceurSubsystem.setDeltaMoyConsigne(38);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mLanceurSubsystem.isAtSetSpeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

}
