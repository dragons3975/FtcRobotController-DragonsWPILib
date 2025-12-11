package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveAutonomusCommand extends Command {

private final DriveSubsystem mDriveSubsystem;
private final double mConsigne;
private double mDistanceInit;

public DriveAutonomusCommand(DriveSubsystem driveSubsystem, double consigne) {
    mDriveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    mConsigne = consigne;
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
   mDistanceInit = mDriveSubsystem.getDistance();
    mDriveSubsystem.drive(0,1,0);
}

@Override
public void execute() {
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    double x;
    if ((mDriveSubsystem.getDistance() - mDistanceInit) >= mConsigne)
        return true;
    else
        return false;
}
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    mDriveSubsystem.stop();
}

}
