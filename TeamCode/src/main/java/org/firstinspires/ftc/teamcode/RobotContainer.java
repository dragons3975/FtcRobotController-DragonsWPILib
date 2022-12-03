package org.firstinspires.ftc.teamcode;

import androidx.annotation.VisibleForTesting;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.TestMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.TestMotorSubsystem;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final VuforiaCurrentGame mVuforiaPOWERPLAY = null;

    private final DriveSubsystem mDriveSubsystem;
    private final DriveCommand mDriveCommand;

    private final TestMotorSubsystem mTestMotorSubsystem;
    private final TestMotorCommand mTestMotorCommand;
    private final AutonomousCommandGroup mAutonomousCommandGroup;
    private final GoToAngleCommand mGoStraight;
    private final GoToAngleCommand mGoLeft;
    private final GoToAngleCommand mGoRight;
    private final GoToAngleCommand mGoBack;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        // Initialize Vuforia

        // Initialize using external web camera.

        // Activate here for camera preview.

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry, mVuforiaPOWERPLAY);
        mTestMotorSubsystem = new TestMotorSubsystem(mHardwareMap, mTelemetry);
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mTestMotorCommand = new TestMotorCommand(mTelemetry, mTestMotorSubsystem, mHardwareMap);
        mAutonomousCommandGroup = new AutonomousCommandGroup(mTelemetry, mDriveSubsystem);
        mGoStraight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 0);
        mGoLeft = new GoToAngleCommand(mTelemetry,mDriveSubsystem,mGamepad1, 270);
        mGoRight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 90);
        mGoBack = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 180);

        configureButtonBindings();
        configureDefaultCommands();
    }

    @Override
    protected void finalize() {
        // Don't forget to deactivate Vuforia before the garbage collector removes the DriveSubsystem from memory
    }

    private void configureButtonBindings() {
        JoystickButton start = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kStart);
        start.onTrue(mAutonomousCommandGroup);

        JoystickButton buttonY = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kY);
        buttonY.onTrue(mGoStraight);

        JoystickButton buttonX = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kX);
        buttonX.onTrue(mGoLeft);
        JoystickButton buttonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        buttonB.onTrue(mGoRight);
        JoystickButton buttonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        buttonA.onTrue(mGoBack);

    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
    }
}
