package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

public class OuvrirPinceCommand extends CommandBase {

    private final PinceSubsystem mPinceSubsystem;
    private final Telemetry mTelemetry;

    public OuvrirPinceCommand(Telemetry telemetry, PinceSubsystem pinceSubsystem){
        mTelemetry = telemetry;
        mPinceSubsystem = pinceSubsystem;

        addRequirements(pinceSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceSubsystem.ouvrir();
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
        return false;
        //Cette commande (infinie) doit etre appelee avec un .withTimeout()
    }
}
