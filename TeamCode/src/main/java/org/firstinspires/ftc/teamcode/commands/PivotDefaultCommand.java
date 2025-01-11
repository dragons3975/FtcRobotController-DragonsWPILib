package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotDefaultCommand extends Command{

    private final PinceSubsystem mPinceSubsystem;

    private final XboxController mxBoxController;

    public PivotDefaultCommand(PinceSubsystem pinceSubsystem, XboxController xboxController) {
        mxBoxController = xboxController;
        mPinceSubsystem = pinceSubsystem;

        addRequirements(pinceSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mPinceSubsystem.IncrementPivotPosition(mxBoxController.getRightY() * 0.01);
        DriverStationJNI.getTelemetry().addData("debug", "");
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

