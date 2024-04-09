package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;
    private final XboxController mXboxController;
    private double mX;
    private double mY;
    private double mZ;

    public DriveCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
        mDriveSubsystem = driveSubsystem;
        mXboxController = xboxController;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mX = -mXboxController.getLeftY();
        mY = mXboxController.getLeftX();
        mZ = mXboxController.getRightX();

        mDriveSubsystem.mecanumDrive(mX, mY, mZ);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
