package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PIDAprilTagCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;
    private final VisionSubsystem mvisionSubsystem;

    private double mAprilPosX = 0;

    private final PIDController mPIDY = new PIDController(Constants.VisionConstants.kPX, 0, 0);

    public PIDAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        mDriveSubsystem = driveSubsystem;
        mvisionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPIDY.setTolerance(0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mAprilPosX = mPIDY.calculate(mDriveSubsystem.getY(), mvisionSubsystem.getAprilTagPosX());
        DriverStationJNI.getTelemetry().addData("PID X OUTPUT TABARNAK", mAprilPosX);

        mDriveSubsystem.mecanumDrive(mAprilPosX, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mPIDY.atSetpoint();
    }
}
