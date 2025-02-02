package org.firstinspires.ftc.teamcode.commands.ExtendCommand;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendDefaultCommand extends Command {

    private final ExtensionSubsystem mExtensionSubsystem;

    private final XboxController mXboxController;


    public ExtendDefaultCommand(ExtensionSubsystem extensionSubsystem, XboxController xboxController) {

        mExtensionSubsystem = extensionSubsystem;

        mXboxController = xboxController;

        addRequirements(extensionSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //mExtensionSubsystem.setConsigne(Constants.ExtensionConstants.kXcmExtend);
        //DriverStationJNI.getTelemetry().addData("test", "");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mExtensionSubsystem.incrementConsigne(-mXboxController.getLeftY() * 35);
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
