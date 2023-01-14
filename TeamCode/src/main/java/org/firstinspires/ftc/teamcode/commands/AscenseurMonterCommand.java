package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class AscenseurMonterCommand extends CommandBase {

    private final AscenseurSubsystem mAscenseurSubsystem;
    private final Telemetry mTelemetry;
    private double mConsigne;

    public AscenseurMonterCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem){
        mTelemetry = telemetry;
        mAscenseurSubsystem = ascenseurSubsystem;

        addRequirements(ascenseurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       // mAscenseurSubsystem.setSetPointAscenseur(mConsigne);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mAscenseurSubsystem.manualOveride(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mAscenseurSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
