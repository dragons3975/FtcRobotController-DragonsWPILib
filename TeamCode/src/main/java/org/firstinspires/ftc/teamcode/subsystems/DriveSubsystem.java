package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.FTC_Gyro;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.MecanumDrive;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private FTC_Gyro mGYRO;

    private final PIDController mPIDx = new PIDController(Constants.PIDxConstants.kP, Constants.PIDxConstants.kI, Constants.PIDxConstants.kD);
    private final PIDController mPIDy = new PIDController(Constants.PIDyConstants.kP, Constants.PIDyConstants.kI, Constants.PIDyConstants.kD);
    private final PIDController mPIDz = new PIDController(Constants.PIDzConstants.kP, Constants.PIDzConstants.kI, Constants.PIDzConstants.kD);

    private int mMode = 1;

    private final DcMotor mFrontLeftMotor;
    private final DcMotor mFrontRightMotor;
    private final DcMotorSimple mBackLeftMotor;
    private final DcMotorSimple mBackRightMotor;

    private MecanumDrive mRobotDrive;

    private boolean mIsPIDxEnabled = false;
    private boolean mIsPIDyEnabled = false;

    private DigitalChannel mTactile;

    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mFrontLeftMotor = mHardwareMap.get(DcMotor.class, "Front left");
        mBackLeftMotor = mHardwareMap.get(DcMotorSimple.class, "Back left");
        mBackRightMotor = mHardwareMap.get(DcMotorSimple.class, "Back right");
        mFrontRightMotor = mHardwareMap.get(DcMotor.class, "Front right");

        mTactile = mHardwareMap.get(DigitalChannel.class, "Tactile Jonction");

        mFrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        mFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRobotDrive = new MecanumDrive(mFrontLeftMotor, mBackLeftMotor, mFrontRightMotor, mBackRightMotor);

        mGYRO = new FTC_Gyro(hardwareMap);

        mPIDz.enableContinuousInput(-180, 180);

        mPIDx.setTolerance(Constants.PIDxConstants.kTolerance);
        mPIDy.setTolerance(Constants.PIDyConstants.kTolerance);
        mPIDz.setTolerance(Constants.PIDzConstants.kTolerance);
    }

    @Override
    public void periodic() {
        if (mIsPIDxEnabled) {
            mX = mPIDx.calculate(getEncoderX());
        }
        if (mIsPIDyEnabled) {
            mY = mPIDy.calculate(getEncoderY());
        }

        mZ = -mPIDz.calculate(mGYRO.getAngle());

        mRobotDrive.driveCartesian(mX, mY, mZ);
        /*mTelemetry.addData("getXaxis", getEncoderY());
        mTelemetry.addData("getYaxis", getEncoderX());
        mTelemetry.addData("Mode", mMode);
        mTelemetry.addData("getTick mFrontLeftMotor", mFrontLeftMotor.getCurrentPosition());
        mTelemetry.addData("getTick mFrontRightMotor", mFrontRightMotor.getCurrentPosition());
        mTelemetry.addData("Z", mZ);*/
        mTelemetry.addData("isCapteurJonctionEnfonce", isCapteurJonctionEnfonce());
    }

    public void drive(double x, double y){
        switch (mMode) {
            case 1:
                mX = -y;
                mY = -x;
                break;
            case 2:
                mX = x;
                mY = -y;
                break;
            case 3:
                mX = y;
                mY = x;
                break;
            case 4:
                mX = -x;
                mY = y;
                break;
        }
    }

    public void setZ (double z) {
        mPIDz.setSetpoint(z);

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
            case -90:
                mMode = 4;
            break;
        }
    }

    public void setSetPointX(double x) {
        double currentPosition = getEncoderX();
        mPIDx.setSetpoint(currentPosition + x);
        mIsPIDxEnabled = true;
    }

    public boolean atSetPointX() {
        return mPIDx.atSetpoint();
    }

    public boolean atSetPointZ() {
        return mPIDz.atSetpoint();
    }

    public void  setSetPointY(double y) {
        double currentPosition = getEncoderY();
        mPIDy.setSetpoint(currentPosition + y);
        mIsPIDyEnabled = true;
    }

    public boolean atSetPointY() {
        return mPIDy.atSetpoint();
    }

    public double getEncoderY() {
        double tickY = (mFrontLeftMotor.getCurrentPosition() - mFrontRightMotor.getCurrentPosition())/2;
        return tickY * Constants.DriveConstants.kCmParTick;
    }

    public double getEncoderX() {
        double tickX = (mFrontLeftMotor.getCurrentPosition() + mFrontRightMotor.getCurrentPosition())/2;
        return tickX * Constants.DriveConstants.kCmParTick;
    }

    public boolean isCapteurJonctionEnfonce() {
        return mTactile.getState() == false;
    }

    public void stop () {
        drive(0, 0);
        mIsPIDyEnabled = false;
        mIsPIDxEnabled = false;
    }

}
