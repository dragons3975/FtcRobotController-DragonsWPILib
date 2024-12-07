package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class ClosePinceExtCommand extends Command {

    private final PinceExtensionSubsystem mPinceExtensionSubsystem;

    public ClosePinceExtCommand(PinceExtensionSubsystem pinceExtensionSubsystem) {

        mPinceExtensionSubsystem = pinceExtensionSubsystem;

        addRequirements(pinceExtensionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceExtensionSubsystem.closePince();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
