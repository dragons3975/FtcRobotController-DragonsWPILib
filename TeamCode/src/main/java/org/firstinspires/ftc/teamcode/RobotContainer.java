package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AutonomousCommands.DriveAutonomusCommand;
import org.firstinspires.ftc.teamcode.AutonomousCommands.TurnAutonomousCommand;
import org.firstinspires.ftc.teamcode.commandGroups.MainAutonomousCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
   // private final TestSubsystem mTestSubsystem = new TestSubsystem();
    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
   // private final LanceurSubsystem mLanceurSubsystem = new LanceurSubsystem();
    //private inal RamasseurSubsystem mRamasseurSubsystem = new RamasseurSubsystem();
    //private final TestRecruesSubsystem mTestRecruesSubsystem = new TestRecruesSubsystem();
    //private final RamasseurCommand mRamasseurCommand = new RamasseurCommand(mRamasseurSubsystem);
    //private final LanceurCommand mLanceurCommand = new LanceurCommand(mLanceurSubsystem);
    //private final LanceurPeriodicCommand mLanceurPeriodicCommand = new LanceurPeriodicCommand(mLanceurSubsystem);
   // private final LanceurPidTestCommand mLanceurPidTestCommand = new LanceurPidTestCommand(mLanceurSubsystem);
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);
    private final MainAutonomousCommand mAuto = new MainAutonomousCommand(mDriveSubsystem);


    //private final LanceurManetteCommand mLanceurManetteCommand = new LanceurManetteCommand(mLanceurSubsystem, mXboxController);
  //  private final OmeletCommand mOmeletCommand = new OmeletCommand(mTestSubsystem);
    //private final BobCommand mBobCommand = new BobCommand(mTestSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
    }

    private void configureButtonBindings() {
        //JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        //buttonA.whileTrue(mLanceurPidTestCommand);
        //JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        //buttonX.toggleOnTrue(mLanceurCommand);
        /*JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mOmeletCommand);*/
        /*JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mLanceurCommand);*/
       // JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
       // buttonA.whileTrue(mLanceurPidTestCommand);

    }

    private void configureDefaultCommands() {
       // mLanceurSubsystem.setDefaultCommand(mLanceurManetteCommand);
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAuto;
    }

    public void stop() {
    }
}
