package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class BrasPositionCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;
    private static double kDt = 0.02;

    private final TrapezoidProfile mProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.00000001, 0.0000000001));

    private TrapezoidProfile.State mGoal;
    private TrapezoidProfile.State mSetPoint;

    private double mRotationInit = 0;

    public BrasPositionCommand(BrasSubsystem brasSubsystem, double goal) {
        mBrasSubsystem = brasSubsystem;

        mGoal = new TrapezoidProfile.State(goal, 0);

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mRotationInit = mBrasSubsystem.getPos();
        mSetPoint = new TrapezoidProfile.State(mRotationInit, 0);
        kDt = 0.02;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mSetPoint = mProfile.calculate(0.2, mSetPoint, mGoal);
        mBrasSubsystem.setTarget(mSetPoint.position);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return mProfile.isFinished(kDt);
    }
}

