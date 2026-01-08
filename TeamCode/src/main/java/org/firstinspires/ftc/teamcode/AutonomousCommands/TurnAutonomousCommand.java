package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnAutonomousCommand extends Command {

private final DriveSubsystem mDriveSubsystem;
private final double mConsigne;
private double mAngleInit;

public TurnAutonomousCommand(DriveSubsystem driveSubsystem, double consigne) {
    mDriveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    mConsigne = consigne;
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
   mAngleInit = mDriveSubsystem.getAngle();
   mDriveSubsystem.drive(0,0,1);
}

@Override
public void execute() {
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return (mDriveSubsystem.getAngle() - mAngleInit) >= mConsigne;
}
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    mDriveSubsystem.stop();
}

}
