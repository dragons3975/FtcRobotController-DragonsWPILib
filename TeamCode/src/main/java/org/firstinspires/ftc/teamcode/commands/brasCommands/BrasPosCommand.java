package org.firstinspires.ftc.teamcode.commands.brasCommands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class BrasPosCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private final double mPos;

    public BrasPosCommand(BrasSubsystem brasSubsystem, double pos) {
        mPos = pos;
        mBrasSubsystem = brasSubsystem;

        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.setTarget(mPos);
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
        return mBrasSubsystem.isRotationAtSetPoint();
    }
}

