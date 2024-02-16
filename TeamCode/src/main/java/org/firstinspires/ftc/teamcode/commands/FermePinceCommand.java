package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class FermePinceCommand extends Command{

    private final PinceSubsystem mPinceSubsystem;

    public FermePinceCommand(PinceSubsystem pinceSubsystem) {
        mPinceSubsystem = pinceSubsystem;

        addRequirements(pinceSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceSubsystem.ferme();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande instantannee
        return true;
    }
}

