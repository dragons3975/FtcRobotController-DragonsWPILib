package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;


public class BrasCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;

    private final XboxController mxBoxController;

    private int mXSpeed;
    private int mZRotation;

    private double targetAvant = 0;

    public BrasCommand(BrasSubsystem brasSubsystem, XboxController xboxController) {
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
        DriverStationJNI.getTelemetry().addData("joystick", mxBoxController.getRightY());
        mBrasSubsystem.incrementTarget(mxBoxController.getRightY() * 6);

        mBrasSubsystem.extention(-mxBoxController.getLeftY());
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

