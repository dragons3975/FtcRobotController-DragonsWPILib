package org.firstinspires.ftc.teamcode.commands.brasCommands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

public class BrasDefaultCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private final XboxController mxBoxController;

    public BrasDefaultCommand(BrasSubsystem brasSubsystem, XboxController xboxController) {
        mBrasSubsystem = brasSubsystem;
        mxBoxController = xboxController;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mBrasSubsystem.incrementTarget(mxBoxController.getRightY() * 120);
        mBrasSubsystem.extention(mxBoxController.getLeftY());
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }
}
