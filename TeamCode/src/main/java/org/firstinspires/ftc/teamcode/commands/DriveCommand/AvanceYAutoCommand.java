package org.firstinspires.ftc.teamcode.commands.DriveCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;

public class AvanceYAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private final double myDist;

    public AvanceYAutoCommand(DriveSubsystem driveSubsystem, double yDist) {
        mDriveSubsystem = driveSubsystem;

        myDist = yDist;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        DriverStationJNI.getTelemetry().addData("autonome", "OUI");
        mDriveSubsystem.mecanumDrivePID(0, myDist);
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
        return (mDriveSubsystem.isAtSetPointy());
    }
}
