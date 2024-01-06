package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;


import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PinceCommandBaisse extends Command{

    private final PinceSubsystem mPinceSubsystem;

    private  final XboxController mXboxControler;

    private double pos = 0;

    public PinceCommandBaisse(PinceSubsystem pinceSubsystem, XboxController xboxController) {
        mPinceSubsystem = pinceSubsystem;
        mXboxControler = xboxController;

        addRequirements(pinceSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       pos += mXboxControler.getLeftY() / 110;
       mPinceSubsystem.ModifInclinaison(pos);
       if (pos >= Constants.ConstantsPince.etenduePince) {
           pos = Constants.ConstantsPince.etenduePince;
       }
       if (pos <= 0) {
            pos = 0;
       }
        DriverStationJNI.getTelemetry().addData("pos", pos);
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

