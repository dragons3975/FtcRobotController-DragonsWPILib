package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;


public class ToggleCameraCommand extends Command {

    private final CameraSubsystem mCameraSubsystem;

    private final XboxController mxBoxController;

    private int mXSpeed;
    private int mZRotation;

    private double targetAvant = 0;

    public ToggleCameraCommand(CameraSubsystem cameraSubsystem, XboxController xboxController) {
        mCameraSubsystem = cameraSubsystem;
        mxBoxController = xboxController;

        addRequirements(cameraSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mCameraSubsystem.toggle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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