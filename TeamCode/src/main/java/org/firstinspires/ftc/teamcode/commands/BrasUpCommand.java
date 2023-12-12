package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class BrasUpCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private final XboxController mxBoxController;

    private int mXSpeed;
    private int mZRotation;

    private double targetAvant = 0;

    public BrasUpCommand(BrasSubsystem brasSubsystem, XboxController xboxController) {
        mBrasSubsystem = brasSubsystem;
        mxBoxController = xboxController;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DriverStationJNI.getTelemetry().addData("Init", "ASDASDAS");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mBrasSubsystem.up();
        DriverStationJNI.getTelemetry().addData("joystick", mxBoxController.getRightY());
       // mBrasSubsystem.incrementTarget(mxBoxController.getRightY() * 2);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie car la commande sera appellée avec un withTimeout()
        // donc elle sera interrompue à la fin du timeout
        return false;
    }
}

