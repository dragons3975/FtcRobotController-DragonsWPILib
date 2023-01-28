package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class AscenseurCommand extends CommandBase {

    private final AscenseurSubsystem mAscenseurSubsystem;
    private final Telemetry mTelemetry;
    private double mConsigne;
    private boolean mForceDown;

    public AscenseurCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem , double consigne, boolean forceDown){
        mTelemetry = telemetry;
        mAscenseurSubsystem = ascenseurSubsystem;
        mConsigne = consigne;
        mForceDown = forceDown;

        addRequirements(ascenseurSubsystem);
    }

    public AscenseurCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem , double consigne){
        this(telemetry, ascenseurSubsystem, consigne, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(mForceDown && mAscenseurSubsystem.getMoyenneAscenseurCm() <= mConsigne){
            mAscenseurSubsystem.setConsigneCm(mAscenseurSubsystem.getMoyenneAscenseurCm());
        } else {
            mAscenseurSubsystem.setConsigneCm(mConsigne);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //mAscenseurSubsystem.stop(); Ne pas arrêter, on laisse le PID corriger
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mAscenseurSubsystem.atSetPoint();
    }
}
