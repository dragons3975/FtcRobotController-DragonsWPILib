package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SetConfigReadyCommand extends Command {

    private final ConfigSubsystem mCon

    private final double mXSpeed, mYSpeed;
    private double mDistanceInit;

    public SetConfigReadyCommand(DriveSubsystem driveSubsystem, double x, double y) {
        mDriveSubsystem = driveSubsystem;

        mXSpeed = x;
        mYSpeed = y;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.drivePIDxy(mXSpeed, mYSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (mXSpeed > 0) {
            return mDriveSubsystem.isAtSetPointx();
        } else {
            return mDriveSubsystem.isAtSetPointy();
        }
    }

}
