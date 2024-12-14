package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
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
        mExtensionSubsystem.setConsigne(Constants.ExtensionConstants.kXcmExtend);
        DriverStationJNI.getTelemetry().addData("test", "");
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
