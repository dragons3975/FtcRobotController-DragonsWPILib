package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;

public class AscenseurSecurityCommand extends CommandBase {

    private final AscenseurSubsystem mAscenseurSubsystem;
    private final Telemetry mTelemetry;
    private boolean mHasMoved = false;
    private double mHasMovedPosition = 80;

    public AscenseurSecurityCommand(Telemetry telemetry, AscenseurSubsystem ascenseurSubsystem){
        mTelemetry = telemetry;
        mAscenseurSubsystem = ascenseurSubsystem;

        addRequirements(ascenseurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mAscenseurSubsystem.manualOveride(-Constants.AscenseurConstants.kAscenseurSecurityPower);
        mHasMoved = false;
        mHasMovedPosition = mAscenseurSubsystem.getMoyenneAscenseurCm() - Constants.AscenseurConstants.kSecurityMinMovedDistance;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!mHasMoved){
            mHasMoved = mAscenseurSubsystem.getMoyenneAscenseurCm() < mHasMovedPosition;
        }
        mTelemetry.addData("hasMoved", mHasMoved);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mAscenseurSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mHasMoved && mAscenseurSubsystem.getVitesse() >= -Constants.AscenseurConstants.kSecurityVitesse;
    }
}
