package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PositionMinBrasCommand extends Command {

    private final PinceBrasSubsystem mPinceBrasSubsystem;

    public PositionMinBrasCommand(PinceBrasSubsystem pinceBrasSubsystem) {

        mPinceBrasSubsystem = pinceBrasSubsystem;


        addRequirements(pinceBrasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceBrasSubsystem.PositionPinceMin();
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
        return true;
    }
}