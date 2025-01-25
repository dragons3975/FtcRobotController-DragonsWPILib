package org.firstinspires.ftc.teamcode.commands.ExtendCommand;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendPositionCommand extends Command {

    private final ExtensionSubsystem mExtensionSubsystem;

    private int mPosition;


    public ExtendPositionCommand(ExtensionSubsystem extensionSubsystem, int position) {

        mExtensionSubsystem = extensionSubsystem;

        mPosition = position;

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
        //if (mXboxController.getLeftTriggerAxis() != 0) {
        //    mExtensionSubsystem.incrementConsigne(mXboxController.getLeftTriggerAxis());
        //} else if (mXboxController.getRightTriggerAxis() != 0) {
        //    mExtensionSubsystem.incrementConsigne(mXboxController.getRightTriggerAxis());
        //}
        mExtensionSubsystem.setConsigne(mPosition);
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
