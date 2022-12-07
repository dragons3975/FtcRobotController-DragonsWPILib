package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class HoverAutoCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private int mConsigneY;

    public HoverAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, int consigneY){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        mConsigneY = consigneY;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.drive(0, 0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return mDriveSubsystem.getEncoderY() >= mConsigneY;
    }
}
