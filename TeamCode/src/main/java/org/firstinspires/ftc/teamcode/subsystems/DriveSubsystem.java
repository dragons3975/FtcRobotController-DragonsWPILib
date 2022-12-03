package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.FTC_Gyro;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.MecanumDrive;
import org.firstinspires.ftc.dragonswpilib.interfaces.Gyro;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.dragonswpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private FTC_Gyro mGYRO;

    private final PIDController mPIDz = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI,Constants.PIDConstants.kD);

    private double mSetPointZ = 0;



    private int mMode = 1;

    private final DcMotor mFrontLeftMotor;
    private final DcMotor mFrontRightMotor;
    private final DcMotor mBackLeftMotor;
    private final DcMotor mBackRightMotor;

    private MecanumDrive mRobotDrive;

    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    private final VuforiaCurrentGame mVuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults mVuforiaResults;

    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, VuforiaCurrentGame vuforiaPOWERPLAY) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mVuforiaPOWERPLAY = vuforiaPOWERPLAY;
        mFrontLeftMotor = mHardwareMap.get(DcMotor.class, "Front left");
        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "Front right");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "Back right");
        mFrontRightMotor = mHardwareMap.get(DcMotor.class, "Back left");


        mFrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        mFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRobotDrive = new MecanumDrive(mFrontLeftMotor, mBackLeftMotor, mFrontRightMotor, mBackRightMotor);


    }



    @Override
    public void periodic() {

        /*if(mSetPointZ == 180) {
            if (angleActuel < 0) {
                angleActuel += 360;
            }
        }
        mTelemetry.addData("angleActuel", angleActuel);
        mZ = mPIDz.calculate(angleActuel);
*/
        mRobotDrive.driveCartesian(mX, mY, -mZ, mGYRO.getRotation2d());


    }

    public void drive(double x, double y, double z){
        switch (mMode) {
            case 1:
                mX = x;
                mY = y;
                break;
            case 2:
                mX = y;
                mY = -x;
                break;
            case 3:
                mX = -y;
                mY = x;
                break;
            case 4:
                mX = -x;
                mY = -y;
        }
            //mZ = z;

    }

    public void setZ (double z) {
        mSetPointZ = z;
        if(mSetPointZ < -180) {
            mSetPointZ = mSetPointZ + 360;
        } else if (mSetPointZ > 180) {
            mSetPointZ = mSetPointZ - 360;
        }
        mPIDz.setSetpoint(mSetPointZ);

        switch ((int)z) {
            case 0 :
                mMode = 1;
            break;
            case 90 :
                mMode = 2;
            break;
            case 180:
                mMode = 3;
            break;
            case 270:
                mMode = 4;
            break;
        }
    }


    public void stop () {
        drive(0, 0, 0);
    }

}

