package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtentionAutoCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private double mExtention;

    private double init;

    public ExtentionAutoCommand(BrasSubsystem brasSubsystem, double extention) {
        mBrasSubsystem = brasSubsystem;
        mExtention = extention;

        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.extention(0.5);
        init = mBrasSubsystem.getExtentionPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBrasSubsystem.extention(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mBrasSubsystem.getExtentionPosition() >= init + mExtention;
    }
}

