package org.firstinspires.ftc.teamcode.commands.DriveCommand;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;

public class AvanceXAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double mXDist;

    private final double mYDist;

    public AvanceXAutoCommand(DriveSubsystem driveSubsystem,double xDist, double yDist) {
        mDriveSubsystem = driveSubsystem;

        mXDist = xDist;
        mYDist = yDist;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.mecanumDrivePID(mXDist, mYDist);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        DriverStationJNI.getTelemetry().addData("autonome", "OUI");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
        mDriveSubsystem.resetDeplacement();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
