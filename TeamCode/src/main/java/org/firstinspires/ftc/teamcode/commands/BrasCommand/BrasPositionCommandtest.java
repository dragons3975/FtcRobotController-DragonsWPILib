package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
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

    private double diference;

    public BrasPositionCommandtest(BrasSubsystem brasSubsystem, double goal, double temps) {
        mBrasSubsystem = brasSubsystem;
        mGoal = goal;
        mTemps = temps;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        total = 0;
        posInit = mBrasSubsystem.getPos();
        diference = mGoal - posInit;
        //mGoal += posInit;
        increments = (diference) / (mTemps * 30);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (increments < 0) {
            if (total + posInit >= mGoal) {
                total += increments;
            }
        } else {
            if (total + posInit <= mGoal) {
                total += increments;
            }
        }
        mBrasSubsystem.setTarget(total + posInit);
        DriverStationJNI.getTelemetry().addData("difference", diference);
        DriverStationJNI.getTelemetry().addData("total - init", total + posInit);
        DriverStationJNI.getTelemetry().addData("total", total);
        DriverStationJNI.getTelemetry().addData("increments", increments);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (abs((total + posInit) - mGoal)) < 5;
    }
}

