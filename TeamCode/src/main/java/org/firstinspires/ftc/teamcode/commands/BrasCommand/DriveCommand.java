package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

private final DriveSubsystem mDriveSubsystem;
private final XboxController mXboxController;

public DriveCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
    mDriveSubsystem = driveSubsystem;
    mXboxController = xboxController;
    addRequirements(driveSubsystem);
}

    //public DriveCommand(DriveSubsystem driveSubsystem) {
    //}

    // Called when the command is initially scheduled.
@Override
public void initialize() {
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    mDriveSubsystem.drive(mXboxController.getLeftX(), -mXboxController.getLeftY(), mXboxController.getRightX());
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    // Commande infinie
    return false;
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    mDriveSubsystem.stop();
}

}
