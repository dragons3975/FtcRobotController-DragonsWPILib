package org.firstinspires.ftc.teamcode.commands.ExtendCommand;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class CalibrationCommand extends Command {

    private final ExtensionSubsystem mExtensionSubsystem;

    private int mTachoInitial = 0;

    public CalibrationCommand(ExtensionSubsystem extensionSubsystem) {

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
        mExtensionSubsystem.StartCalibration();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mExtensionSubsystem.CalibrationZero();
        mExtensionSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mExtensionSubsystem.isCalibrationFinished();
    }
}
