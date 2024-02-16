package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class TourneAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final int mRotation;


    public TourneAutoCommand(DriveSubsystem driveSubsystem, int rotation) {
        mDriveSubsystem = driveSubsystem;

        mRotation = rotation;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.mecanumDrive(0, 0, mRotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mDriveSubsystem.isAtSetPoint();
    }
}
