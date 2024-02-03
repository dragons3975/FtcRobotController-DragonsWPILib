package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveAutoCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    private double mXSpeed;
    private double mZRotation;

    private double mYSpeed;

    public DriveAutoCommand(DriveSubsystem driveSubsystem, double x, double z, double y) {
        mDriveSubsystem = driveSubsystem;

        mXSpeed = x;
        mZRotation = z;
        mYSpeed = y;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //mDriveSubsystem.mecanumDrive(mXSpeed, mZRotation, mYSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie car la commande sera appellée avec un withTimeout()
        // donc elle sera interrompue à la fin du timeout
        return false;
    }
}
