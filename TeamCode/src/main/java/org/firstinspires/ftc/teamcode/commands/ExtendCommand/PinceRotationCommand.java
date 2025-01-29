package org.firstinspires.ftc.teamcode.commands.ExtendCommand;

import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PinceRotationCommand extends Command{

    private final PinceExtensionSubsystem mPinceExtensionSubsystem;
    private double mAngle;

    public PinceRotationCommand(PinceExtensionSubsystem pinceExtensionSubsystem, double angle) {
        mPinceExtensionSubsystem = pinceExtensionSubsystem;
        mAngle = angle;

        addRequirements(pinceExtensionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceExtensionSubsystem.setRotationAngle(mAngle);
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
        // Commande infinie
        return true;
    }
}

