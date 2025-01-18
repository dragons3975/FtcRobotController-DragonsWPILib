package org.firstinspires.ftc.teamcode.commands;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.XboxController;


import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LiftUpCommand extends Command {

    private final XboxController mXboxController2;

    private final LiftSubsystem mLiftSubsystem;

    public LiftUpCommand(LiftSubsystem liftSubsystem, XboxController xboxController2) {
        mXboxController2 = xboxController2;
        mLiftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }
    public void execute() {
        mLiftSubsystem.incrementTargetRotation(1);
    }

    @Override
    public void initialize() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
