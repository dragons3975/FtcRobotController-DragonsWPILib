package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class CallibrateAscenseurCommand extends CommandBase {

    private final AscenseurSubsystem mAscenseurSubsystem;
    private final Telemetry mTelemetry;
    private double mConsigne;

    public CallibrateAscenseurCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem){
        mTelemetry = telemetry;
        mAscenseurSubsystem = ascenseurSubsystem;

        addRequirements(ascenseurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mAscenseurSubsystem.manualOveride(-0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mAscenseurSubsystem.stop();
        mAscenseurSubsystem.calibrate();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mAscenseurSubsystem.isLeftDown() && mAscenseurSubsystem.isRightDown();

    }
}
