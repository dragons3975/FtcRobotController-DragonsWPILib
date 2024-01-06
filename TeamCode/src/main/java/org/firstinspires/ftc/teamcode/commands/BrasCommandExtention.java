package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class BrasCommandExtention extends Command{

    private  final BrasSubsystem mBrasSubsystem;

    private  final XboxController mXboxControler;

    private double mExt = 0;

    public BrasCommandExtention(BrasSubsystem brasSubsystem, XboxController xboxController, double ext) {
        mExt = ext;
        mBrasSubsystem = brasSubsystem;
        mXboxControler = xboxController;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.extention(mExt);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBrasSubsystem.extention(0);


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

