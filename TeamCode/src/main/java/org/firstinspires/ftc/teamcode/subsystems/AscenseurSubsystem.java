package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class AscenseurSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotor mMoteurAscenseurGauche;
    private final DcMotor mMoteurAscenseurDroit;
    private final PIDController mPIDMoyenne;
    private final PIDController mPIDDiff;

    private boolean mPIDenabled = false;
    private double mPowerLeft = 0;
    private double mPowerRight = 0;

    private double mCalibrationLeftTick;
    private double mCalibrationRightTick;

    private DigitalChannel mDigitalInputLeft;
    private DigitalChannel mDigitalInputRight;

    private boolean mIsCalibrated = false;

    public AscenseurSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurAscenseurGauche = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur gauche");
        mMoteurAscenseurDroit = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur droit");

        mDigitalInputLeft = mHardwareMap.get(DigitalChannel.class, "LeftTactile");
        mDigitalInputRight = mHardwareMap.get(DigitalChannel.class, "RightTactile");

        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mMoteurAscenseurGauche.setDirection(DcMotor.Direction.FORWARD);

        mMoteurAscenseurDroit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseurDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mMoteurAscenseurDroit.setDirection(DcMotor.Direction.REVERSE);

        mPIDMoyenne = new PIDController(Constants.PIDascenseurConstants.kPMoy, Constants.PIDascenseurConstants.kIMoy, Constants.PIDascenseurConstants.kDMoy);
        mPIDMoyenne.setTolerance(Constants.PIDascenseurConstants.kMoyTolerance);

        mPIDDiff = new PIDController(Constants.PIDascenseurConstants.kPDiff, Constants.PIDascenseurConstants.kIDiff, Constants.PIDascenseurConstants.kDDiff);
        mPIDDiff.setTolerance(Constants.PIDascenseurConstants.kDiffTolerance);
        mPIDDiff.setSetpoint(0);
    }

    @Override
    public void periodic() {
        if(mPIDenabled && mIsCalibrated) {
            double outputPidMoyenne = mPIDMoyenne.calculate(getMoyenneAscenseurCm());
            double outputPidDiff = mPIDDiff.calculate(getMotorDiffCm());

            mTelemetry.addData("mPIDMoyenne.getSetpoint()", mPIDMoyenne.getSetpoint());
            mTelemetry.addData("getMoyenneAscenseurCm", getMoyenneAscenseurCm());
            mTelemetry.addData("getMotorDiffCm", getMotorDiffCm());

            mTelemetry.addData("outputPidMoyenne", outputPidMoyenne);
            mTelemetry.addData("outputPidDiff", outputPidDiff);

            mPowerLeft = outputPidMoyenne + outputPidDiff;
            mPowerRight = outputPidMoyenne - outputPidDiff;

            double maximumPower = Math.max(Math.abs(mPowerLeft), Math.abs(mPowerRight));
            if (maximumPower > 1){
                mPowerLeft = mPowerLeft / maximumPower ;
                mPowerRight = mPowerRight / maximumPower;
            }
        }
        if (isLeftDown() && mPowerLeft < 0) {
            mPowerLeft = 0;
        }
        if (isRightDown() && mPowerRight < 0) {
            mPowerRight = 0;
        }
        mMoteurAscenseurGauche.setPower(mPowerLeft);
        mMoteurAscenseurDroit.setPower(mPowerRight);

        /*mTelemetry.addData("mPowerLeft", mPowerLeft);
        mTelemetry.addData("mPowerRight", mPowerRight);*/

        mTelemetry.addData("Ascenseur Position", getMoyenneAscenseurCm());
        /*mTelemetry.addData("mCalibrationLeftTick", mCalibrationLeftTick);
        mTelemetry.addData("mCalibrationRightTick", mCalibrationRightTick);*/


        mTelemetry.addData("isLeftDown", isLeftDown());
        mTelemetry.addData("isRightDown", isRightDown());
    }

    private double getAscenseurGauchePositionCm() {
        return (mMoteurAscenseurGauche.getCurrentPosition() - mCalibrationLeftTick) * Constants.AscenseurConstants.kCmParTick;
    }

    private double getAscenseurDroitPositionCm() {
        return (mMoteurAscenseurDroit.getCurrentPosition() - mCalibrationRightTick) * Constants.AscenseurConstants.kCmParTick;
    }

    private double getMoyenneAscenseurCm(){
        return (getAscenseurDroitPositionCm() + getAscenseurGauchePositionCm())/2;
    }

    private double getMotorDiffCm() {
       return getAscenseurGauchePositionCm() - getAscenseurDroitPositionCm();
    }

    public void setDeltaConsigneCm(double consigne) {
        double consigneCm = mPIDMoyenne.getSetpoint() + consigne;
        if (consigneCm > Constants.AscenseurConstants.kPositionMax){
            consigneCm = Constants.AscenseurConstants.kPositionMax;
        }
        if (consigneCm < Constants.AscenseurConstants.kPositionMin){
            consigneCm = Constants.AscenseurConstants.kPositionMin;
        }
        setConsigneCm(consigneCm);
    }

    public void setConsigneCm(double consigne) {
        mPIDMoyenne.setSetpoint(consigne);
        mPIDenabled = true;
    }

    public boolean atSetPoint() {
        return mPIDMoyenne.atSetpoint() && mPIDDiff.atSetpoint();
    }

    private boolean isLeftDown(){
        return mDigitalInputLeft.getState() == false;
    }

    private boolean isRightDown() {
        return mDigitalInputRight.getState() == false;
    }

    public boolean isDown() {
        return isLeftDown() && isRightDown();
    }

    public void manualOveride(double power) {
        mPowerLeft = power;
        mPowerRight = power;
        mPIDenabled = false;
    }

    public void moteurGaucheManualOveride(double power) {
        mPowerLeft = power;
        mPowerRight = 0;
        mPIDenabled = false;
    }

    public void calibrate() {
        mCalibrationLeftTick = mMoteurAscenseurGauche.getCurrentPosition();
        mCalibrationRightTick = mMoteurAscenseurDroit.getCurrentPosition();
        setConsigneCm(0);
        mIsCalibrated = true;
    }

    public void stop() {
        setConsigneCm(getMoyenneAscenseurCm());
        mPowerLeft = 0;
        mPowerRight = 0;
        mPIDenabled = false;
    }

}
