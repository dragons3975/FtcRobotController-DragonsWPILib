package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;

public class AvanceAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double mXSpeed, mDistance;
    private double mDistanceInit;

    public AvanceAutoCommand(DriveSubsystem driveSubsystem, double x, int distance) {
        mDriveSubsystem = driveSubsystem;

        mDistance = distance;

        mXSpeed = x;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDistanceInit = mDriveSubsystem.getX();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.mecanumDrive(mXSpeed, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (mXSpeed > 0 && mDriveSubsystem.getX() > mDistanceInit + mDistance)
                || (mXSpeed < 0 && mDriveSubsystem.getX() < mDistanceInit - mDistance);
    }

}
