package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DeactivatePropPipelineCommand extends Command{
    VisionSubsystem mVisionSubsystem;
    public DeactivatePropPipelineCommand(VisionSubsystem visionSubsystem) {
        mVisionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mVisionSubsystem.deactivateProp();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
