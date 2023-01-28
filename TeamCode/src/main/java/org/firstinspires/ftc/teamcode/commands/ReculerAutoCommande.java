package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ReculerAutoCommande extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private int mDistance;
    private double mVitesse;
    private double mpositionactuel = 0;
    private double reculerdistance = 50;

    public ReculerAutoCommande(Telemetry telemetry, DriveSubsystem driveSubsystem, int distance, double vitesse){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        mDistance = distance;


        mVitesse = vitesse;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mpositionactuel = mDriveSubsystem.getPositionCm();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.ArcadeDrive(-mVitesse, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mDriveSubsystem.getPositionCm() >= mpositionactuel + reculerdistance;
    }
}
