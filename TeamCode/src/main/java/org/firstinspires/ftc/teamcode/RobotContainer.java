package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.BrasIncrCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    //private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasIncrCommand mIncrBrasPlus = new BrasIncrCommand(mBrasSubsystem, Constants.BrasIncr.Bras, 0,0);
    private final BrasIncrCommand mIncrBrasMinus = new BrasIncrCommand(mBrasSubsystem, -Constants.BrasIncr.Bras, 0,0);
    private final BrasIncrCommand mIncrCoudePlus = new BrasIncrCommand(mBrasSubsystem, 0, Constants.BrasIncr.Coude,0);
    private final BrasIncrCommand mIncrCoudeMinus = new BrasIncrCommand(mBrasSubsystem, 0, -Constants.BrasIncr.Coude,0);
    private final BrasIncrCommand mIncrRotationPlus = new BrasIncrCommand(mBrasSubsystem, 0, 0,Constants.BrasIncr.Rotation);
    private final BrasIncrCommand mIncrRotationMinus = new BrasIncrCommand(mBrasSubsystem, 0, 0,-Constants.BrasIncr.Rotation);
    private final PinceCommand mMovePince = new PinceCommand(mPinceSubsystem);




    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mBrasSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mIncrBrasPlus);

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mIncrBrasMinus);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mIncrCoudePlus);

        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.onTrue(mIncrCoudeMinus);

        JoystickButton DpadUp = new JoystickButton(mXboxController, XboxController.Button.kUp.value);
        DpadUp.onTrue(mIncrRotationPlus);

        JoystickButton DpadDown = new JoystickButton(mXboxController, XboxController.Button.kDown.value);
        DpadDown.onTrue(mIncrRotationMinus);

        JoystickButton DpadRight = new JoystickButton(mXboxController, XboxController.Button.kRight.value);
        DpadRight.onTrue(mMovePince);

    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}
