package org.firstinspires.ftc.teamcode.commands.brasCommands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class BrasExtentionPosCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private double mExtention;

    public BrasExtentionPosCommand(BrasSubsystem brasSubsystem, double extention) {
        mBrasSubsystem = brasSubsystem;
        mExtention = extention;

        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.setTargetExtention(mExtention);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBrasSubsystem.stopExtention();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mBrasSubsystem.isExtentionAtSetPoint();
    }
}

