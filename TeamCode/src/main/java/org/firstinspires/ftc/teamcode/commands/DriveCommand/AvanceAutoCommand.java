package org.firstinspires.ftc.teamcode.commands.DriveCommand;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AvanceAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double mXSpeed;
    private final double mXDist;

    public AvanceAutoCommand(DriveSubsystem driveSubsystem, double xSpeed, double xDist) {
        mDriveSubsystem = driveSubsystem;

        mXDist = xDist;
        mXSpeed = xSpeed;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.mecanumDrive(mXSpeed, 0, 0);
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
        return (abs(mDriveSubsystem.getCurrentPose().getX()) > mXDist);
    }
}
