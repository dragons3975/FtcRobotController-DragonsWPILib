package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Objects;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;

public class TasseAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double mYSpeed, mDistance;
    private double DistanceInitiale = 0;

    public TasseAutoCommand(DriveSubsystem driveSubsystem, double y, int distance) {
        mDriveSubsystem = driveSubsystem;

        mDistance = distance;

        mYSpeed = y;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DistanceInitiale = mDriveSubsystem.getY();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.mecanumDrive(0, mYSpeed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (mYSpeed > 0 && mDriveSubsystem.getY() > DistanceInitiale + mDistance) || (mYSpeed < 0 && mDriveSubsystem.getY() < DistanceInitiale - mDistance);
    }

}
