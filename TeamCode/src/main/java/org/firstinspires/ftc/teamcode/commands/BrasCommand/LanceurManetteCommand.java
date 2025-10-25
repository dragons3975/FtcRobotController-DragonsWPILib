package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class LanceurManetteCommand extends Command {

    private final LanceurSubsystem mLanceurSubsystem;
    private final XboxController mXboxController;

    public LanceurManetteCommand(LanceurSubsystem lanceurSubsystem, XboxController xboxController) {
        mLanceurSubsystem = lanceurSubsystem;
        mXboxController = xboxController;
        addRequirements(lanceurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mLanceurSubsystem.setSpeed(mXboxController.getLeftY());
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
