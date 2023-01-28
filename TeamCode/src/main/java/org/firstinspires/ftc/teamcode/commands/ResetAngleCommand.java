package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ResetAngleCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;

    public ResetAngleCommand(Telemetry telemetry, DriveSubsystem driveSubsystem){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.resetZ();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
