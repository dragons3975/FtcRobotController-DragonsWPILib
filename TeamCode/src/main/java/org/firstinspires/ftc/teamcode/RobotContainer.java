package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.BrasPosition0;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche.RougeGaucheTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.CalibreExtentionCommand;
import org.firstinspires.ftc.teamcode.commands.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceToggleDroitCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceToggleGaucheCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceToggleInclinaisonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceToggleCommand;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final ConfigSubsystem mConfigSubsystem = new ConfigSubsystem();
    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
    //public final TeamPropPipeline mTeamPropPipeline = new TeamPropPipeline();

    private final PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();
    private final LanceurSubsystem mLanceurSubsystem = new LanceurSubsystem();

    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);
    private final BrasPosCommand mBrasCommandPositionMilieu = new BrasPosCommand(mBrasSubsystem, Constants.BrasConstants.kRotationMilieu);

    /*private final ToggleVisionPipelineCommand mToggleVisionPipelineCommand = new ToggleVisionPipelineCommand(mVisionSubsystem);

    private final ActivatePropPipelineCommand mActivatePropPipelineCommand = new ActivatePropPipelineCommand(mVisionSubsystem);
    private final ActivateAprilTagPipelineCommand mActivateAprilTagPipelineCommand = new ActivateAprilTagPipelineCommand(mVisionSubsystem);

    private final DeactivatePropPipelineCommand mDeactivatePropPipelineCommand = new DeactivatePropPipelineCommand(mVisionSubsystem);

    private final DeactivateAprilTagPipelineCommand mDeactivateAprilTagPipelineCommand = new DeactivateAprilTagPipelineCommand(mVisionSubsystem);*/

    private final PinceToggleCommand mPinceCommandToggle = new PinceToggleCommand(mPinceSubsystem);
    private final LanceurCommand mlanceurCommand = new LanceurCommand(mLanceurSubsystem);

//    private final ToggleAllianceColorCommand mToggleAllianceColorCommand = new ToggleAllianceColorCommand(mConfigSubsystem);
//    private final ToggleAlliancePositionCommand mToggleAlliancePositionCommand = new ToggleAlliancePositionCommand(mConfigSubsystem);
//    private final ToggleTemporaryTeamPropPositionCommand mToggleTemporaryTeamPropPositionCommand = new ToggleTemporaryTeamPropPositionCommand(mConfigSubsystem);

    private final PinceToggleDroitCommand mPinceCommandToggleDroit = new PinceToggleDroitCommand(mPinceSubsystem);
    private final PinceToggleGaucheCommand mPinceCommandToggleGauche = new PinceToggleGaucheCommand(mPinceSubsystem);
    private final PinceToggleInclinaisonCommand mPinceToggleInclinaisonCommand = new PinceToggleInclinaisonCommand(mPinceSubsystem);
    private final CalibreExtentionCommand mCalibreExtentionCommand = new CalibreExtentionCommand(mBrasSubsystem);
    private final BrasPosition0 mBrasPosition0AutoCommand = new BrasPosition0(mBrasSubsystem, mPinceSubsystem);

//    private final LeveBras mLeveBras = new LeveBras(mBrasSubsystem, mPinceSubsystem);


    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {

        JoystickButton kUp = new JoystickButton(mXboxController, XboxController.Button.kUp.value);
        //kUp.onTrue(mActivatePropPipelineCommand);

        JoystickButton kRight = new JoystickButton(mXboxController, XboxController.Button.kRight.value);
        //kRight.onTrue(mActivateAprilTagPipelineCommand);

        JoystickButton kDown = new JoystickButton(mXboxController, XboxController.Button.kDown.value);
        //kDown.onTrue(mDeactivateAprilTagPipelineCommand);

        JoystickButton kLeft = new JoystickButton(mXboxController, XboxController.Button.kLeft.value);
        //kLeft.onTrue(mDeactivatePropPipelineCommand);
        ///////////////////////////////////////////////////////////////////
        JoystickButton kStart = new JoystickButton(mXboxController, XboxController.Button.kStart.value);
        //kStart.onTrue(mToggleAllianceColorCommand);

        JoystickButton kBack = new JoystickButton(mXboxController, XboxController.Button.kBack.value);
        //kBack.onTrue(mToggleAlliancePositionCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        //buttonY.onTrue(mToggleVisionPipelineCommand);
        buttonY.onTrue(mlanceurCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        //buttonB.onTrue(mBrasPosition0AutoCommand);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.onTrue(mBrasPosition0AutoCommand);

        JoystickButton buttonRB2 = new JoystickButton(mXboxController2, XboxController.Button.kRightBumper.value);
        buttonRB2.onTrue(mBrasCommandPositionMilieu);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mPinceCommandToggle);

        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY2.onTrue(mCalibreExtentionCommand);

        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA2.onTrue(mPinceToggleInclinaisonCommand);

        JoystickButton buttonkLeft2 = new JoystickButton(mXboxController2, XboxController.Button.kLeft.value);
        buttonkLeft2.onTrue(mPinceCommandToggleGauche);

        JoystickButton buttonkRight2 = new JoystickButton(mXboxController2, XboxController.Button.kRight.value);
        buttonkRight2.onTrue(mPinceCommandToggleDroit);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasDefaultCommand);
    }

    public Command getAutonomousCommand() {

        DriverStationJNI.getTelemetry().addData("couleur", mConfigSubsystem.allianceColor());
        DriverStationJNI.getTelemetry().addData("position", mConfigSubsystem.alliancePosition());
        DriverStationJNI.getTelemetry().addData("TeamPropLocation", mVisionSubsystem.getTeamPropLocation());

        return new RougeGaucheTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
        /*
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kBleu) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuGaucheTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuGaucheTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuGaucheTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuDroiteTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuDroiteTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuDroiteTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kRouge) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeGaucheTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeGaucheTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeGaucheTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeDroiteTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeDroiteTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeDroiteTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }
        return null;*/
    }
}
