package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.dragonswpilib.drive.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.CallibrateAscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.TestGabrielCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.testGabriel;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DriveSubsystem mDriveSubsystem;
    private final DriveCommand mDriveCommand;
    private final TestGabrielCommand mTestGabrielCommand;
    private final testGabriel mTestGabriel;
    private final AscenseurSubsystem mAscenseurSubsystem;

    private final AutonomousCommandGroup mAutonomousCommandGroup;
    private final GoToAngleCommand mGoStraight;
    private final GoToAngleCommand mGoLeft;
    private final GoToAngleCommand mGoRight;
    private final GoToAngleCommand mGoBack;
    private final CallibrateAscenseurCommand mCallibrateAscenseurCommand;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mAscenseurSubsystem = new AscenseurSubsystem(mHardwareMap, mTelemetry);
        mTestGabriel = new testGabriel(mHardwareMap, mTelemetry);
        mTestGabrielCommand = new TestGabrielCommand(mTelemetry, mTestGabriel, mGamepad1);
        mCallibrateAscenseurCommand = new CallibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);
        mCallibrateAscenseurCommand.schedule();

        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mAutonomousCommandGroup = new AutonomousCommandGroup(mTelemetry, mDriveSubsystem);
        mGoStraight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 0);
        mGoLeft = new GoToAngleCommand(mTelemetry,mDriveSubsystem,mGamepad1, 90);
        mGoRight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, -90);
        mGoBack = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 180);

        configureButtonBindings();
        configureDefaultCommands();
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
        mTestGabriel.setDefaultCommand(mTestGabrielCommand);

    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
    }
}
