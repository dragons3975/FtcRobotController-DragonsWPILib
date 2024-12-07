package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendCommand extends Command {

    private final ExtensionSubsystem mExtensionSubsystem;


    public ExtendCommand(ExtensionSubsystem extensionSubsystem) {

        mExtensionSubsystem = extensionSubsystem;

        addRequirements(extensionSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mExtensionSubsystem.setSpeed(0.75);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mExtensionSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mExtensionSubsystem.getDistanceXcm()>=100;
    }
}
