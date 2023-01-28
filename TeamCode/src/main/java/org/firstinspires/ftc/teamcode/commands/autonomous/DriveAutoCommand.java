package org.firstinspires.ftc.teamcode.commands.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveAutoCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private final double mConsigneY;
    private final double mConsigneX;
    private final double mMaxSpeed;

    public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, double consigneX, double consigneY, double maxSpeed) {
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        mMaxSpeed = maxSpeed;

        mConsigneX = consigneX;
        mConsigneY = consigneY * Constants.DriveConstants.kHoverBoost;

        addRequirements(driveSubsystem);
    }

    public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, double consigneX, double consigneY) {
        this(telemetry, driveSubsystem, consigneX, consigneY, Constants.DriveConstants.kMaxOutput);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.setSetPointY(mConsigneY);
        mDriveSubsystem.setSetPointX(mConsigneX);

        mDriveSubsystem.setMaxSpeed(mMaxSpeed);
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
        return (mDriveSubsystem.atSetPointY() && mDriveSubsystem.atSetPointX());
    }
}
