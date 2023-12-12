package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CalibreBrasCommand extends Command {

    private final BrasSubsystem mBrasSubsystem;

    public CalibreBrasCommand(BrasSubsystem brasSubsystem) {
        mBrasSubsystem = brasSubsystem;

        addRequirements(mBrasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.calibreActif();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBrasSubsystem.calibreDesactif();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mBrasSubsystem.verifButton();
    }
}
