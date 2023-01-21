package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.dragonswpilib.math.MathUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class AscenseurDefaultDeltaCommand extends CommandBase {

    private final AscenseurSubsystem mAscenseurSubsystem;
    private final Telemetry mTelemetry;
    private final Gamepad mGamepad;

    public AscenseurDefaultDeltaCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem, Gamepad gamepad){
        mTelemetry = telemetry;
        mAscenseurSubsystem = ascenseurSubsystem;
        mGamepad = gamepad;

        addRequirements(ascenseurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mAscenseurSubsystem.setDeltaConsigneCm(-(MathUtil.applyDeadband(mGamepad.right_stick_y, 0.1)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //mAscenseurSubsystem.stop(); Ne pas arrêter, on laisse le PID corriger
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
