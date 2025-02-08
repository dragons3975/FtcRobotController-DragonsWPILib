package org.firstinspires.ftc.teamcode.commands.ExtendCommand;

import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PincePositionEchangeExtCommand extends Command {

    private final PinceExtensionSubsystem mPinceExtensionSubsystem;
    private double mPos;

    public PincePositionEchangeExtCommand(PinceExtensionSubsystem pinceExtensionSubsystem, double pos) {

        mPinceExtensionSubsystem = pinceExtensionSubsystem;
        mPos = pos;


        addRequirements(pinceExtensionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceExtensionSubsystem.setPositionAngle(mPos);
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