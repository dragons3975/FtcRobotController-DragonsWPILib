package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.testGabriel;

public class TestGabrielCommand extends CommandBase {

    private final testGabriel mtestGabriel;
    private final Telemetry mTelemetry;
    private final Gamepad mGamepad;

    public TestGabrielCommand(Telemetry telemetry, testGabriel test, Gamepad gamepad){
        mTelemetry = telemetry;
        mtestGabriel = test;
        mGamepad = gamepad;

        addRequirements(test);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(mGamepad.right_trigger > 0)
            mtestGabriel.yann(mGamepad.right_trigger);
        if(mGamepad.left_trigger > 0)
            mtestGabriel.yann(-mGamepad.left_trigger);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mtestGabriel.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
