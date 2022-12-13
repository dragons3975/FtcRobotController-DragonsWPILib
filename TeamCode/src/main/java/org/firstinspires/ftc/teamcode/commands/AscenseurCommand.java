package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class AscenseurCommand extends CommandBase {

    private final AscenseurSubsystem mBrasSubsystem;
    private final Telemetry mTelemetry;
    private final Gamepad mGamepad;
    private double mConsigne;

    public AscenseurCommand(Telemetry telemetry, AscenseurSubsystem brasSubsystem, Gamepad gamepad, double consigne){
        mTelemetry = telemetry;
        mBrasSubsystem = brasSubsystem;
        mGamepad = gamepad;
        mConsigne = consigne;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.setSetPointAscenseur(mConsigne);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBrasSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mBrasSubsystem.atSetPointAscenseur();
    }
}
