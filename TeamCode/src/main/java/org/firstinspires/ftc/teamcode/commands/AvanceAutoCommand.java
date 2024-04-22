package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;

public class AvanceAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double mXSpeed, mYSpeed;
    private double mDistanceInit;

    public AvanceAutoCommand(DriveSubsystem driveSubsystem, double x, double y) {
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
        if (mXSpeed != 0) {
            return mDriveSubsystem.isAtSetPointx();
        } else {
            return mDriveSubsystem.isAtSetPointy();
        }
    }

}
