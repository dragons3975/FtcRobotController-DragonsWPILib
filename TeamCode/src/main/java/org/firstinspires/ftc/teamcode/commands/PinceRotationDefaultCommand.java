package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PinceRotationDefaultCommand extends Command{

    private final PinceExtensionSubsystem mPinceExtensionSubsystem;

    private final XboxController mxBoxController;

    public PinceRotationDefaultCommand(PinceExtensionSubsystem pinceExtensionSubsystem, XboxController xboxController) {
        mPinceExtensionSubsystem = pinceExtensionSubsystem;
        mxBoxController = xboxController;

        addRequirements(pinceExtensionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mPinceExtensionSubsystem.setRotationAngle(mxBoxController.getLeftX());
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }
}

