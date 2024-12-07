package org.firstinspires.ftc.teamcode.commands;

import static android.os.Environment.getExternalStorageDirectory;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.DecimalFormat;
import java.util.ArrayList;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class TrajectoryCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;
    private final Trajectory mTrajectory;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    private final HolonomicDriveController mController = new HolonomicDriveController(
            new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0),
            new ProfiledPIDController(0.1, 0, 0,
                    new TrapezoidProfile.Constraints(6.28, 3.14)));

    private Trajectory generateTrajectory() {
        Pose2d sideStart = new Pose2d(0, 0,
                Rotation2d.fromDegrees(0));
        Pose2d crossScale = new Pose2d(50, 0,
                Rotation2d.fromDegrees(90));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(33, 0));
        interiorWaypoints.add(new Translation2d(66, 0));

        TrajectoryConfig config = new TrajectoryConfig(20 , 10);
        config.setReversed(false);

        return TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }

    private double mCurrentTime = 0;
    private Writer fileWriter;
    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+ "trajectory";

    public TrajectoryCommand(DriveSubsystem driveSubsystem) {
        mDriveSubsystem = driveSubsystem;
        mController.setTolerance(new Pose2d(5, 5, new Rotation2d(Math.toRadians(5))));

        addRequirements(driveSubsystem);



        mTrajectory = generateTrajectory();
        File directory = new File(directoryPath);
        directory.mkdir();


        try {
            fileWriter = new FileWriter(directoryPath+"/emerik.csv");
            for ( double i = 0.2 ; i < mTrajectory.getTotalTimeSeconds(); i+=0.2) {
                fileWriter.write(df.format(mTrajectory.sample(i).timeSeconds) + ",");
                fileWriter.write(df.format(mTrajectory.sample(i).poseMeters.getX()) + ",");
                fileWriter.write(df.format(mTrajectory.sample(i).poseMeters.getY()) + ",");
                fileWriter.write(df.format(mTrajectory.sample(i).poseMeters.getRotation().getDegrees()));
                fileWriter.write("\n");
            }
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mCurrentTime = 0;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mCurrentTime += 0.02;
        Trajectory.State goal = mTrajectory.sample(mCurrentTime);

        ChassisSpeeds adjustedSpeeds = mController.calculate(mDriveSubsystem.getCurrentPose(), goal, Rotation2d.fromDegrees(0));

        //MecanumDriveKinematics test = new MecanumDriveKinematics(new Translation2d(0,0),new Translation2d(0,0),new Translation2d(0,0),new Translation2d(0,0));
        //MecanumDriveWheelSpeeds speeds = test.toWheelSpeeds(adjustedSpeeds);


        DriverStationJNI.getTelemetry().addData("x speed", adjustedSpeeds.vxMetersPerSecond);
        DriverStationJNI.getTelemetry().addData("y speed", adjustedSpeeds.vyMetersPerSecond);
        DriverStationJNI.getTelemetry().addData("z speed",adjustedSpeeds.omegaRadiansPerSecond);
        DriverStationJNI.getTelemetry().addData("is at reference", mController.atReference());

        mDriveSubsystem.mecanumDrive(adjustedSpeeds.vxMetersPerSecond / 15, adjustedSpeeds.vyMetersPerSecond / 15, adjustedSpeeds.omegaRadiansPerSecond / 10);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//mController.atReference();
    }
}
