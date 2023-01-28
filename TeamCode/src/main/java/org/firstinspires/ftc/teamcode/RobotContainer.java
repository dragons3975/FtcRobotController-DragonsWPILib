package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.Autonomous2CommandGroupDroite;
import org.firstinspires.ftc.teamcode.commandGroups.Autonomous2CommandGroupGauche;
import org.firstinspires.ftc.teamcode.commandGroups.Autonomous2CommandGroupMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupDroite;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupGauche;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroupMilieu;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.LiftDown_CMD;
import org.firstinspires.ftc.teamcode.commands.LiftStopped_CMD;
import org.firstinspires.ftc.teamcode.commands.LiftUp_CMD;
import org.firstinspires.ftc.teamcode.commands.TagDetection_CMD;
import org.firstinspires.ftc.teamcode.commands.servoClose_CMD;
import org.firstinspires.ftc.teamcode.commands.servoOpen_CMD;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagsDetection_SS;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.Lift_SS;
import org.firstinspires.ftc.teamcode.subsystems.Servo_SS;

public class RobotContainer {

    private  Gamepad mGamepad1, mGamepad2;
    private  Telemetry mTelemetry;
    private  HardwareMap mHardwareMap;
    private  DriveSubsystem mDriveSubsystem;
    private  DriveCommand mDriveCommand;
    private  AprilTagsDetection_SS m_AprilTagsDetection_SS;

    private TagDetection_CMD mTagDetection_CMD;

    private  Lift_SS m_Lift_SS;
    private  LiftUp_CMD  m_LiftUp_CMD;
    private  LiftDown_CMD  m_LiftDown_CMD;
    private  LiftStopped_CMD m_LiftStopped_CMD;
    private final double  liftUpSpeed = 0.4;
    private final double  liftSpeedDown = -0.4;


    private  Servo_SS m_servo_SS;
    private  servoOpen_CMD m_servoOpen_CMD;
    private  servoClose_CMD m_servoClose_CMD;

    private final double  servoOpenDegrees = 90;
    private final double  servoCloseDegrees = 0;

    private int mPosition;

    private Autonomous2CommandGroupGauche mAutonomous2CommandGroupGauche;
    private Autonomous2CommandGroupDroite mAutonomous2CommandGroupDroite;
    private Autonomous2CommandGroupMilieu mAutonomous2CommandGroupMilieu;
    private AutonomousCommandGroupDroite mAutonomousCommandGroupDroite;
    private AutonomousCommandGroupGauche mAutonomousCommandGroupGauche;
    private AutonomousCommandGroupMilieu mAutonomousCommandGroupMilieu;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;


        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mAutonomous2CommandGroupGauche = new Autonomous2CommandGroupGauche(mTelemetry, mDriveSubsystem);
        mAutonomous2CommandGroupDroite = new Autonomous2CommandGroupDroite(mTelemetry, mDriveSubsystem);
        mAutonomous2CommandGroupMilieu = new Autonomous2CommandGroupMilieu(mTelemetry, mDriveSubsystem);
        mAutonomousCommandGroupDroite = new AutonomousCommandGroupDroite(mTelemetry, mDriveSubsystem);
        mAutonomousCommandGroupGauche = new AutonomousCommandGroupGauche(mTelemetry, mDriveSubsystem);
        mAutonomousCommandGroupMilieu = new AutonomousCommandGroupMilieu(mTelemetry, mDriveSubsystem);

        m_AprilTagsDetection_SS = new AprilTagsDetection_SS(mHardwareMap, mTelemetry);

        mTagDetection_CMD  = new TagDetection_CMD(mTelemetry, m_AprilTagsDetection_SS);

        m_Lift_SS = new Lift_SS(mHardwareMap, mTelemetry);
        m_LiftUp_CMD  = new LiftUp_CMD(mTelemetry,m_Lift_SS, mGamepad1,liftUpSpeed );
        m_LiftDown_CMD  = new LiftDown_CMD(mTelemetry,m_Lift_SS, mGamepad1,liftSpeedDown );
        m_LiftStopped_CMD = new LiftStopped_CMD(mTelemetry,m_Lift_SS, mGamepad1 );

       m_servo_SS = new Servo_SS(mHardwareMap, mTelemetry);
       m_servoOpen_CMD = new servoOpen_CMD(mTelemetry,m_servo_SS, mGamepad1 );
       m_servoClose_CMD= new servoClose_CMD(mTelemetry,m_servo_SS, mGamepad1 );

        //mPosition = m_AprilTagsDetection_SS.readTags();


        configureButtonBindings();
        configureDefaultCommands();
    }

    public RobotContainer() {
    }

    @Override
    protected void finalize() {
    }

    private void configureButtonBindings() {
        //JoystickButton DPadUp = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kDpadUp);
       // DPadUp.onTrue(mAutonomousCommandGroupDroite);

        JoystickButton ButtonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        ButtonB.whileTrue(m_LiftUp_CMD);

        JoystickButton ButtonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        ButtonA.whileTrue(m_LiftDown_CMD);

        JoystickButton ButtonX = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kX);
        ButtonX.whileTrue(m_servoOpen_CMD);

        JoystickButton ButtonY = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kY);
        ButtonY.whileTrue(m_servoClose_CMD);


    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
       // m_Lift_SS.setDefaultCommand(m_LiftStopped_CMD);
    }

    public Command getAutonomousCommand() {

        if( m_AprilTagsDetection_SS.readTags() == 1 )
        {
            return mAutonomousCommandGroupGauche;

        } else if ( m_AprilTagsDetection_SS.readTags() == 2 ) {
            return mAutonomousCommandGroupMilieu;

        }  else if ( m_AprilTagsDetection_SS.readTags() == 3 )  {

            return mAutonomousCommandGroupDroite;

        }
        else {

            return mAutonomousCommandGroupMilieu;
        }





       /* switch(mPosition) {
            case 1:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGauche;
            case 2:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupMilieu;
            case 3:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupDroite;
            default:
                mTelemetry.addData("choix Tag ModeAuto:", mPosition);
                return mAutonomousCommandGroupMilieu;
        }*/
    }


    /*public Command getAutonomousCommand2() {

        if( m_AprilTagsDetection_SS.readTags() == 1 )
        {
            return mAutonomous2CommandGroupGauche;

        } else if ( m_AprilTagsDetection_SS.readTags() == 2 ) {
            return mAutonomous2CommandGroupMilieu;

        }  else if ( m_AprilTagsDetection_SS.readTags() == 3 )  {

            return mAutonomous2CommandGroupDroite;

        }
        else {

            return mAutonomous2CommandGroupMilieu;
        }*/








}


