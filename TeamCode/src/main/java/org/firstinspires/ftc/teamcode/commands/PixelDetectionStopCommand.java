package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.subsystems.PixelDetectionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PixelDetectionStopCommand extends Command{

    private final PixelDetectionSubsystem mPixelDetectionSubsystem;

    public PixelDetectionStopCommand(PixelDetectionSubsystem pixelDetectionSubsystem) {
        mPixelDetectionSubsystem = pixelDetectionSubsystem;

        addRequirements(pixelDetectionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPixelDetectionSubsystem.stopCamera();
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

