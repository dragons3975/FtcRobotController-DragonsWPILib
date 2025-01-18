package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class LiftDownCommand extends Command {

    private final XboxController mXboxController2;

    private final LiftSubsystem mLiftSubsystem;

    public LiftDownCommand(LiftSubsystem liftSubsystem, XboxController xboxController2) {
        mXboxController2 = xboxController2;
        mLiftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }
    public void execute() {
        mLiftSubsystem.incrementTargetRotation(-30);
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
        return false;
    }
}
