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

    private double mAngleConsigne = 0;

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
    }

    public void drive(double x, double y) {
        double angleActuelRadians = Math.toRadians(mGYRO.getAngle());
        mX = x * Math.sin(angleActuelRadians) - y * Math.cos(angleActuelRadians);
        mY = -y * Math.sin(angleActuelRadians) - x * Math.cos(angleActuelRadians);
    }

    public void setZ (double z) {
        if(Math.abs(z) > 0.1) {
            mAngleConsigne = z * 5;//mAngleConsigne + (z*3);
            double current = mGYRO.getAngle();
            mPIDz.setSetpoint(-mAngleConsigne + current);
        }
    }

    public boolean atSetPointZ() {
        return mPIDz.atSetpoint();
    }

    public void setSetPointX(double x) {
        double currentPosition = getEncoderX();
        mPIDx.setSetpoint(currentPosition + x);
        mIsPIDxEnabled = true;
    }

    public boolean atSetPointX() {
        return mPIDx.atSetpoint();
    }

    public void  setSetPointY(double y) {
        double currentPosition = getEncoderY();
        mPIDy.setSetpoint(currentPosition + y);
        mIsPIDyEnabled = true;
    }

    public boolean atSetPointY() {
        return mPIDy.atSetpoint();
    }

    private double getEncoderY() {
        double tickY = (mFrontLeftMotor.getCurrentPosition() - mFrontRightMotor.getCurrentPosition())/2;
        return tickY * Constants.DriveConstants.kCmParTick;
    }

    private double getEncoderX() {
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
