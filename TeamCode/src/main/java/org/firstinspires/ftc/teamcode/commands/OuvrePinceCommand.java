package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class OuvrePinceCommand extends Command{

    private final PinceSubsystem mPinceSubsystem;

    public OuvrePinceCommand(PinceSubsystem pinceSubsystem) {
        mPinceSubsystem = pinceSubsystem;

        addRequirements(pinceSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceSubsystem.ouvre();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande instantannee
        return true;
    }
}

