package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos2;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.commands.PinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.commands.ToggleCameraCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);

    private final BrasCommandPos2 mBrasCommandPos2 = new BrasCommandPos2(mBrasSubsystem, 330);


   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);

    private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);

    private final  PinceCommand mPinceCommand = new PinceCommand(mPinceSubsystem);

    private final CalibreBrasCommand mCalibreBrasCommand = new CalibreBrasCommand(mBrasSubsystem);

    private final CameraSubsystem mCameraSubsystem = new CameraSubsystem();

    private  final ToggleCameraCommand mToggleCameraCommand = new ToggleCameraCommand(mCameraSubsystem, mXboxController);

    //private final AvancerAutoCommand mAvancerAutoCommand = new AvancerAutoCommand(mDriveSubsystem, 0, 0, 0);

    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem);;

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton buttonX = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX.whileTrue(mBrasCommandPos1);

        JoystickButton buttonY = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY.whileTrue(mBrasCommandPos2);

        JoystickButton buttonA = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA.onTrue(mCalibreBrasCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mIntakeCommand);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mToggleCameraCommand);

    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
        //mCameraSubsystem.setDefaultCommand(mToggleCameraCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}
