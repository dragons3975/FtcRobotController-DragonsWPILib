package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveAutoCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private int mConsigneY;
    private int mConsigneX;
    private String Bonjour;


    public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, int consigneX, int consigneY){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;

        mConsigneX = consigneX;
        mConsigneY = consigneY;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {


        mTelemetry.addData("Command Active", true);

        mDriveSubsystem.setSetPointY(mConsigneY);
        mDriveSubsystem.setSetPointX(mConsigneX);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mTelemetry.addData("DriveAutoCommand", Bonjour);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mTelemetry.addData("Command Active", false);
        mDriveSubsystem.stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        mTelemetry.addData("At setpoint y", mDriveSubsystem.atSetPointY());
        mTelemetry.addData("At setpoint x", mDriveSubsystem.atSetPointX());
        boolean isFinished = mDriveSubsystem.atSetPointY() && mDriveSubsystem.atSetPointX();
        return isFinished;// ***|| mDriveSubsystem.getEncoder for X (needs math to calculate mecanum) ***
    }
}
