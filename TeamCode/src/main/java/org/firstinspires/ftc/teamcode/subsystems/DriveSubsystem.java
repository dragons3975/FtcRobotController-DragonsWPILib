package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.FTC_Gyro;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.MecanumDrive;
import org.firstinspires.ftc.dragonswpilib.interfaces.Gyro;
import org.firstinspires.ftc.dragonswpilib.math.MathUtil;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

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
    private boolean mIsPIDzEnabled = true;

    private DigitalChannel mTactile;

    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    private final File mCalibrationFile;

    private double mZOffset = 0;

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
        String filename = "gyro_calibration.json";
        mCalibrationFile = AppUtil.getInstance().getSettingsFile(filename);

        mPIDz.enableContinuousInput(-180, 180);

        mPIDx.setTolerance(Constants.PIDxConstants.kTolerance);
        mPIDy.setTolerance(Constants.PIDyConstants.kTolerance);
        mPIDz.setTolerance(Constants.PIDzConstants.kTolerance);
        setMaxSpeed(1.0);
    }

    @Override
    public void periodic() {
        if (mIsPIDxEnabled) {
            mX = mPIDx.calculate(getEncoderX());
        }
        if (mIsPIDyEnabled) {
            mY = mPIDy.calculate(getEncoderY());
        }

        if (mIsPIDzEnabled) {
            mZ = -mPIDz.calculate(getZ());
        }
        mRobotDrive.driveCartesian(mX, mY, mZ);
        // mTelemetry.addData("mZ", mZ);
    }

    private double getZ() {
        return MathUtil.inputModulus(mGYRO.getAngle() - mZOffset, -180, 180);
    }

    public void drive(double x, double y) {
        double angleActuelRadians = Math.toRadians(getZ());
        mX = x * Math.sin(angleActuelRadians) - y * Math.cos(angleActuelRadians);
        mY = -y * Math.sin(angleActuelRadians) - x * Math.cos(angleActuelRadians);
    }

    public void setZ (double z) {
        if(Math.abs(z) > 0.1) {
            double angleConsigne = z * 5;
            double current = getZ();
            mPIDz.setSetpoint(-angleConsigne + current);
        }
        mTelemetry.addData("z", z);
    }
    public void setZAutonomous (double z) { //set Z est une incrimentation, ne marche pas avec Autonomous
           // if(z == 0) {
                mIsPIDzEnabled = true;
                mPIDz.setSetpoint(z);
            /*} else{
            mAngleConsigne = z;
            double current = mGYRO.getAngle();
            mPIDz.setSetpoint(mAngleConsigne + current);}*/
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

    public void disablePIDz() {
        mIsPIDzEnabled = false;
    }

    public void setMaxSpeed(double speed) {
        mRobotDrive.setMaxOutput(speed);
    }

    public void stop () {
        drive(0, 0);
        mIsPIDyEnabled = false;
        mIsPIDxEnabled = false;
        setMaxSpeed(1.0);
    }

    public void resetZ() {
        mZOffset = mGYRO.getAngle();
        if (mIsPIDzEnabled) {
            setZAutonomous(getZ());
        }
    }

    /*private boolean isGyroCalibrationFileAvailable() {
        return mCalibrationFile.lastModified() > (System.currentTimeMillis() - SystemClock.elapsedRealtime());
    }

    private void readGyroCalibration() {
        JsonParser jsonParser = new JsonParser();
        try {
            FileReader reader = new FileReader(mCalibrationFile);
            JsonObject json = jsonParser.parse(reader).getAsJsonObject();
            mAngleConsigne = json.get("angleConsigne").getAsDouble();
        } catch (FileNotFoundException ignored) {
        }
    }

    private void writeGyroCalibration() {
        JsonObject json = new JsonObject();
        json.addProperty("angleConsigne", mAngleConsigne);
        try {
            mCalibrationFile.createNewFile();
            mCalibrationFile.setWritable(true);
            FileWriter filewriter = new FileWriter(mCalibrationFile);
            filewriter.write(json.toString());
            filewriter.flush();
        } catch (IOException e) {
        }
    }*/



}
