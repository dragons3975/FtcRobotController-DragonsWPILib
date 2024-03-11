package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BoucheSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class BoucheCommand extends Command{

    private final BoucheSubsystem mBoucheSubsystem;

    public BoucheCommand(BoucheSubsystem BoucheSubsystem){
        mBoucheSubsystem =BoucheSubsystem;
        addRequirements(BoucheSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBoucheSubsystem.power=1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mBoucheSubsystem.power=0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

