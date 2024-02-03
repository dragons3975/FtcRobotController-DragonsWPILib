package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class GoToPositionBrasCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;
    private final int mPosition;

    public GoToPositionBrasCommand(BrasSubsystem brasSubsystem, int position) {
        mBrasSubsystem = brasSubsystem;
        mPosition = position;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.goToPosition(mPosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
