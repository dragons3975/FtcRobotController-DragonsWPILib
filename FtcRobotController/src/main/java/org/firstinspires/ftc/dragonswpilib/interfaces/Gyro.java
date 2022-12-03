package org.firstinspires.ftc.dragonswpilib.interfaces;

import org.firstinspires.ftc.dragonswpilib.math.geometry.Rotation2d;

public interface Gyro  {
    void calibrate();

    /**
     * Reset the gyro. Resets the gyro to a heading of zero. This can be used if there is significant
     * drift in the gyro and it needs to be recalibrated after it has been running.
     */
    void reset();

    /**
     * Return the heading of the robot in degrees.
     *
     * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
     * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
     * 360 to 0 on the second time around.
     *
     * <p>The angle is expected to increase as the gyro turns clockwise when looked at from the top.
     * It needs to follow the NED axis convention.
     *
     * <p>This heading is based on integration of the returned rate from the gyro.
     *
     * @return the current heading of the robot in degrees.
     */
    double getAngle();

    /**
     * Return the rate of rotation of the gyro.
     *
     * <p>The rate is based on the most recent reading of the gyro analog value
     *
     * <p>The rate is expected to be positive as the gyro turns clockwise when looked at from the top.
     * It needs to follow the NED axis convention.
     *
     * @return the current rate in degrees per second
     */
    double getRate();

    /**
     * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
     *
     * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
     * top. It needs to follow the NWU axis convention.
     *
     * <p>This heading is based on integration of the returned rate from the gyro.
     *
     * @return the current heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
     */
    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getAngle());
    }
}

