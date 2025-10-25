package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

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
        mLanceurSubsystem.monte();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mLanceurSubsystem.stop();
    }

}
