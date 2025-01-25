package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BrasPositionCommandtest extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private double increments;

    private double total;

    private double mGoal;

    private double mTemps;

    private double posInit;

    public BrasPositionCommandtest(BrasSubsystem brasSubsystem, double goal, double temps) {
        mBrasSubsystem = brasSubsystem;
        mGoal = goal;
        mTemps = temps;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        posInit = mBrasSubsystem.getPos();
        mGoal -= posInit;
        increments = (mGoal) / (mTemps * 30);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        total += increments;
        if (increments < 0) {
            if (total <= mGoal) {
                total = mGoal;
            }
        } else {
            if (total >= mGoal) {
                total = mGoal;
            }
        }
        mBrasSubsystem.setTarget(total  + posInit);
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

