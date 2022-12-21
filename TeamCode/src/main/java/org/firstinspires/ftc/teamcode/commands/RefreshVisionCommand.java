package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.testGabriel;

public class RefreshVisionCommand extends CommandBase {

    private final Telemetry mTelemetry;
    private final VisionSubsystem mVisionSubsystem;

    public RefreshVisionCommand(Telemetry telemetry, VisionSubsystem visionSubsystem){
        mTelemetry = telemetry;
        mVisionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mVisionSubsystem.setAutonomousPosition(1);
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
