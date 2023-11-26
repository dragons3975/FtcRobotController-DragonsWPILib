package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AvancerCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private double mXSpeed;
    private double mZRotation;

    private double mDistance;

    private double mYSpeed;

    public AvancerCommand(DriveSubsystem driveSubsystem, double x, double z, double y, double distance) {
        mDriveSubsystem = driveSubsystem;
        mDistance = distance;

        mXSpeed = x;
        mZRotation = z;
        mYSpeed = y;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.mecanumDrive(mXSpeed, mZRotation, mYSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (mDriveSubsystem.getDistanceX() >= mDistance);
    }
}
