package com.ftc12835.roverruckus.subsystems;

import android.support.annotation.Nullable;
import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.library.hardware.CachingDcMotorEx;
import com.acmerobotics.library.hardware.LynxOptimizedI2cFactory;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.motion.MotionConstraints;
import com.acmerobotics.library.motion.PIDController;
import com.acmerobotics.library.motion.PIDFCoefficients;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.library.path.TrajectoryBuilder;
import com.acmerobotics.library.path.TrajectoryFollower;
import com.acmerobotics.library.telemetry.TelemetryUtil;
import com.acmerobotics.library.util.DrawingUtil;
import com.ftc12835.roverruckus.localization.Localizer;
import com.ftc12835.roverruckus.opmodes.auto.AutoOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

/**
 * Tried to base a differential drive off of ACME's mecanum drive subsystem, but a lot of the cooler
 * things don't work with a non-holonomic drive, so RIP. (also the localizer is broken now so
 * don't try to use it)
 *
 * Differential drive kinematics paper: http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
 *
 * Wheel layout (top view):
 *
 *        FRONT
 * (1)||---------||(3)
 *      |       |
 *      |       |
 *      |       |
 *      |       |
 * (2)||---------||(4)
 *
 */
@Config
public class DifferentialDrive extends Subsystem {
    public static final int IMU_PORT = 0;

    public static final int TRACKING_ENCODER_TICKS_PER_REV = 2000;

    public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12);
    public static final PIDCoefficients SLOW_VELOCITY_PID = new PIDCoefficients(10, 3, 1);

    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(30.0, 40.0, 160.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 2.67, 10.67, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PIDF = new PIDFCoefficients(-0.5, 0, 0, 0.230, 0);
    public static PIDFCoefficients AXIAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0177, 0);
    public static PIDFCoefficients LATERAL_PIDF = new PIDFCoefficients(-0.05, 0, 0, 0.0179, 0);

    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(-2, 0, -0.01);

    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PATH
    }

    public static final String[] MOTOR_NAMES = {"frontLeft", "rearLeft", "frontRight", "rearRight"};

    public static final double WHEELBASE_WIDTH = 18;
    public static final double WHEELBASE_HEIGHT = 18;

    /**
     * K = (wheelbase width + wheelbase height) / 4 (in)
     */
    public static final double K = (WHEELBASE_WIDTH + WHEELBASE_HEIGHT) / 4;

    /**
     * wheel radius (in)
     */
    public static final double RADIUS = 4;

    private DcMotorEx[] motors;

    /**
     * units in encoder ticks; solely intended for internal use
     */
    private int[] driveEncoderOffsets;
    private boolean useCachedDriveEncoderPositions;
    private int[] cachedDriveEncoderPositions;

    private int[] trackingEncoderOffsets;
    private boolean useCachedTrackingEncoderPositions;
    private int[] cachedTrackingEncoderPositions;

    private BNO055IMU imu;

    private boolean useCachedOrientation;
    private Orientation cachedOrientation;
    private double headingOffset;

    private boolean positionEstimationEnabled;
    private Pose2d estimatedPose = new Pose2d(0, 0, 0);

    private TrajectoryFollower trajectoryFollower;

    private Localizer localizer;

    private PIDController maintainHeadingController;
    private boolean maintainHeading;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers;
    private Vector2d targetVel = new Vector2d(0, 0);
    private double targetOmega = 0;

    private LynxModule frontHub, rearHub;

    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode driveMode;

        public double frontLeftPower;
        public double rearLeftPower;
        public double rearRightPower;
        public double frontRightPower;

        public boolean positionEstimationEnabled;
        public double estimatedX;
        public double estimatedY;
        public double estimatedHeading;

        public boolean maintainHeading;
        public double maintainHeadingError;
        public double maintainHeadingUpdate;

        public double pathAxialError;
        public double pathAxialUpdate;
        public double pathHeadingError;
        public double pathHeadingUpdate;
    }

    public DifferentialDrive(HardwareMap map) {
        this.telemetryData = new TelemetryData();

        frontHub = map.get(LynxModule.class, "frontHub");
        rearHub = map.get(LynxModule.class, "rearHub");

        I2cDeviceSynch imuI2cDevice = LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(frontHub, IMU_PORT);
        imuI2cDevice.setUserConfiguredName("imu");
        imu = new LynxEmbeddedIMU(imuI2cDevice);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        /*
        // taken from https://ftcforum.usfirst.org/forum/ftc-technology/53812-mounting-the-revhub-vertically?p=56587#post56587
        // testing suggests that an axis remap is more accurate then simply changing the axis read
        // we hypothesize that this helps properly configure the internal sensor fusion/Kalman filtering
        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
            byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        */

        powers = new double[4];
        driveEncoderOffsets = new int[4];
        trackingEncoderOffsets = new int[4];
        motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i ++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = new CachingDcMotorEx(dcMotorEx);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[4].setDirection(DcMotorSimple.Direction.REVERSE);
        setVelocityPIDCoefficients(NORMAL_VELOCITY_PID);

//        localizer = new DeadReckoningLocalizer(this);
        setEstimatedPose(estimatedPose);

        trajectoryFollower = new TrajectoryFollower(HEADING_PIDF, AXIAL_PIDF, LATERAL_PIDF);
        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);
        maintainHeadingController.setInputBounds(-Math.PI, Math.PI);

        resetDriveEncoders();
        resetTrackingEncoders();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        localizer.setEstimatedPosition(estimatedPose.pos());
    }

    public TrajectoryFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }

    public void enablePositionEstimation() {
        positionEstimationEnabled = true;
    }

    public void disablePositionEstimation() {
        positionEstimationEnabled = false;
    }

    public DcMotorEx[] getMotors() {
        return motors;
    }

    public void enableHeadingCorrection(double desiredHeading) {
        maintainHeading = true;
        this.maintainHeadingController.setSetpoint(desiredHeading);
        this.maintainHeadingController.reset();
    }

    public void disableHeadingCorrection() {
        maintainHeading = false;
    }

    public void setTargetHeading(double heading) {
        this.maintainHeadingController.setSetpoint(heading);
    }

    public boolean getMaintainHeading() {
        return maintainHeading;
    }

    private void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setHeading(double heading) {
        headingOffset = -getRawHeading() + heading;
    }

    // Diff. drive doesn't allow for lateral movement, so the x value of the velocity vector is
    // useless, but idc enough to refactor this
    public void setVelocity(Vector2d vel, double omega) {
        internalSetVelocity(vel, omega);
        mode = Mode.OPEN_LOOP;
    }

    private void internalSetVelocity(Vector2d vel, double omega) {
        this.targetVel = vel;
        this.targetOmega = omega;
    }

    private void updatePowers() {
        powers[0] = targetVel.y() - targetOmega;
        powers[1] = targetVel.y() - targetOmega;
        powers[2] = targetVel.y() + targetOmega;
        powers[3] = targetVel.y() + targetOmega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }
    }

    public void stop() {
        setVelocity(new Vector2d(0, 0), 0);
    }

    /**
     * get distance traveled from encoder values
     *
     * [W] is wheel rotations
     * [V] = [vX, vY, vOmega]'
     * [F] = [ 1/4,    1/4,   1/4,   1/4]
     *       [-1/4,    1/4,  -1/4,   1/4]
     *       [-1/4K, -1/4K,  1/4K,  1/4K]
     *
     * [V] = (r)[W][F]
     *
     * im 99% sure this won't work with a diff drive btw
     *
     * @param rot rotation of each wheel, in radians
     * @return movement of robot
     */
    public static Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double x = RADIUS * ( rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = RADIUS * (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * DifferentialDrive.K);
        return new Pose2d(x, y, h);
    }

    private void resetTrackingEncoders() {
        int[] positions = internalGetTrackingEncoderPositions();
        for(int i = 0; i < 2; i++) {
            trackingEncoderOffsets[i] = -positions[i];
        }
    }

    public double[] getTrackingEncoderRotations() {
        double[] motorRotations = new double[2];
        int[] encoderPositions = getTrackingEncoderPositions();
        for (int i = 0; i < 2; i++) {
            motorRotations[i] = trackingEncoderTicksToRadians(encoderPositions[i]);
        }
        return motorRotations;
    }

    public int[] getTrackingEncoderPositions() {
        if (!useCachedTrackingEncoderPositions || cachedTrackingEncoderPositions == null) {
            cachedTrackingEncoderPositions = internalGetTrackingEncoderPositions();
            for (int i = 0; i < 2; i++) {
                cachedTrackingEncoderPositions[i] += trackingEncoderOffsets[i];
            }
            useCachedTrackingEncoderPositions = true;
        }
        return cachedTrackingEncoderPositions;
    }

    private int[] internalGetTrackingEncoderPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(rearHub);
        int[] positions = new int[2];
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            for (int i = 0; i < 2; i++) {
                positions[i] = response.getEncoder(i);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.w("MecanumDrive", e);
        }
        return positions;
    }

    private void resetDriveEncoders() {
        int[] positions = internalGetDriveEncoderPositions();
        for(int i = 0; i < 4; i++) {
            driveEncoderOffsets[i] = -positions[i];
        }
    }

    /** @return motor rotations in radians */
    public double[] getDriveMotorRotations() {
        double[] motorRotations = new double[4];
        int[] encoderPositions = getDriveEncoderPositions();
        for (int i = 0; i < 4; i++) {
            motorRotations[i] = driveEncoderTicksToRadians(encoderPositions[i]);
        }
        return motorRotations;
    }

    /** @return motor positions in encoder ticks */
    public int[] getDriveEncoderPositions() {
        if (!useCachedDriveEncoderPositions || cachedDriveEncoderPositions == null) {
            cachedDriveEncoderPositions = internalGetDriveEncoderPositions();
            for (int i = 0; i < 4; i++) {
                cachedDriveEncoderPositions[i] += driveEncoderOffsets[i];
            }
            useCachedDriveEncoderPositions = true;
        }
        return cachedDriveEncoderPositions;
    }

    private int[] internalGetDriveEncoderPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(frontHub);
        int[] positions = new int[4];
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            for (int i = 0; i < 4; i++) {
                positions[i] = response.getEncoder(i);
            }
            positions[2] = -positions[2];
            positions[3] = -positions[3];
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.w("MecanumDrive", e);
        }
        return positions;
    }

    private double driveEncoderTicksToRadians(int ticks) {
        double ticksPerRev = motors[0].getMotorType().getTicksPerRev();
        return 2 * Math.PI * ticks / ticksPerRev;
    }

    private double trackingEncoderTicksToRadians(int ticks) {
        return 2 * Math.PI * ticks / TRACKING_ENCODER_TICKS_PER_REV;
    }

    private Orientation internalGetAngularOrientation() {
        return imu.getAngularOrientation();
    }

    public Orientation getAngularOrientation() {
        if (!useCachedOrientation || cachedOrientation == null) {
            cachedOrientation = internalGetAngularOrientation();
            useCachedOrientation = true;
        }
        return cachedOrientation;
    }

    private double getRawHeading() {
        return getAngularOrientation().firstAngle;
    }

    public double getHeading() {
        return Angle.norm(getRawHeading() + headingOffset);
    }

    public void resetHeading() {
        setHeading(0);
    }

    public void followTrajectory(Trajectory trajectory) {
        if (!positionEstimationEnabled) {
            throw new IllegalStateException("Position estimation must be enable for path following");
        }
        trajectoryFollower.follow(trajectory);
        setMode(Mode.FOLLOW_PATH);
    }

    public boolean isFollowingTrajectory() {
        return trajectoryFollower.isFollowingTrajectory();
    }

    public void waitForTrajectoryFollower() {
        while (!Thread.currentThread().isInterrupted() && mode == Mode.FOLLOW_PATH) {
            try {
                Thread.sleep(AutoOpMode.POLL_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void waitForMarker(String name) {
        while (!Thread.currentThread().isInterrupted() && mode == Mode.FOLLOW_PATH) {
            if (trajectoryFollower.getTrajectoryTime() >= trajectoryFollower.getTrajectory().getMarkerTime(name)) {
                return;
            }
            try {
                Thread.sleep(AutoOpMode.POLL_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public Vector2d getEstimatedPosition() {
        return estimatedPose.pos();
    }

    public void setEstimatedPosition(Vector2d position) {
        estimatedPose = new Pose2d(position, estimatedPose.heading());
        localizer.setEstimatedPosition(position);
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setEstimatedPose(Pose2d pose) {
        setEstimatedPosition(pose.pos());
        setHeading(pose.heading());
    }

    private void invalidateCaches() {
        useCachedOrientation = false;
        useCachedDriveEncoderPositions = false;
        useCachedTrackingEncoderPositions = false;
    }

    public void setVelocityPIDCoefficients(PIDCoefficients pidCoefficients) {
        for (int i = 0; i < 4; i++) {
            motors[i].setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d pose) {
        return new TrajectoryBuilder(pose, AXIAL_CONSTRAINTS, POINT_TURN_CONSTRAINTS);
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        invalidateCaches();

        telemetryData.driveMode = mode;
        telemetryData.positionEstimationEnabled = positionEstimationEnabled;
        telemetryData.maintainHeading = maintainHeading;

        // position estimation
        if (positionEstimationEnabled) {
            estimatedPose = new Pose2d(localizer.update(), getHeading());
        }

        switch (mode) {
            case OPEN_LOOP:
                break;
            case FOLLOW_PATH:
                if (trajectoryFollower.isFollowingTrajectory()) {
                    Pose2d estimatedPose = getEstimatedPose();
                    Pose2d update = trajectoryFollower.update(estimatedPose);
                    internalSetVelocity(update.pos(), update.heading());

                    telemetryData.pathAxialError = trajectoryFollower.getAxialError();
                    telemetryData.pathAxialUpdate = trajectoryFollower.getAxialUpdate();
                    telemetryData.pathHeadingError = trajectoryFollower.getHeadingError();
                    telemetryData.pathHeadingUpdate = trajectoryFollower.getHeadingUpdate();
                } else {
                    stop();
                }
                break;
        }

        // maintain heading
        if (maintainHeading) {
            double heading = getHeading();
            double headingError = maintainHeadingController.getError(heading);
            double headingUpdate = maintainHeadingController.update(headingError);
            internalSetVelocity(targetVel, headingUpdate);

            telemetryData.maintainHeadingError = headingError;
            telemetryData.maintainHeadingUpdate = headingUpdate;
        } else {
            internalSetVelocity(targetVel, targetOmega);
        }

        updatePowers();

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }

        telemetryData.frontLeftPower = powers[0];
        telemetryData.rearLeftPower = powers[1];
        telemetryData.rearRightPower = powers[2];
        telemetryData.frontRightPower = powers[3];


        telemetryData.estimatedX = estimatedPose.x();
        telemetryData.estimatedY = estimatedPose.y();
        telemetryData.estimatedHeading = estimatedPose.heading();

        if (fieldOverlay != null) {
            if (trajectoryFollower.getTrajectory() != null) {
                fieldOverlay.setStroke("#4CAF50");
                DrawingUtil.drawTrajectory(fieldOverlay, trajectoryFollower.getTrajectory());
            }

            if (trajectoryFollower.getPose() != null) {
                fieldOverlay.setStroke("#F44336");
                DrawingUtil.drawMecanumRobot(fieldOverlay, trajectoryFollower.getPose());
            }

            fieldOverlay.setStroke("#3F51B5");
            DrawingUtil.drawMecanumRobot(fieldOverlay, estimatedPose);
        }

        return TelemetryUtil.objectToMap(telemetryData);
    }
}
