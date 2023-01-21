package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveAutoCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private int mConsigneY;
    private int mConsigneX;
    //private boolean mUseCapteur = false;

    public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, int consigneX, int consigneY){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;

        mConsigneX = consigneX;
        mConsigneY = consigneY;

        addRequirements(driveSubsystem);
    }

    /*public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, int consigneX, int consigneY, boolean useCapteur){
        this(telemetry, driveSubsystem, consigneX, consigneY);
        mUseCapteur = useCapteur;
    }*/

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.setSetPointY(mConsigneY);
        mDriveSubsystem.setSetPointX(mConsigneX);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mTelemetry.addData("Drive auto command", "on");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
        mTelemetry.addData("Drive auto command", "off");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (mDriveSubsystem.atSetPointY() && mDriveSubsystem.atSetPointX());// || (mUseCapteur && mDriveSubsystem.isCapteurJonctionEnfonce());
    }
}
