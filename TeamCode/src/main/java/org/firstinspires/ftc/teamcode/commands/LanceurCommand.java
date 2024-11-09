package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LanceurCommand extends Command{
    LanceurSubsystem mLanceurSubsystem;
    public LanceurCommand(LanceurSubsystem lanceurSubsystem) {
        mLanceurSubsystem = lanceurSubsystem;
        addRequirements(lanceurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mLanceurSubsystem.lance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie car la commande sera appellée avec un withTimeout()
        // donc elle sera interrompue à la fin du timeout
        return true;
    }
}
