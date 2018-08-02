package com.ftc12835.roverruckus.localization;

import com.acmerobotics.library.localization.Vector2d;
import com.ftc12835.roverruckus.subsystems.DifferentialDrive;
import com.ftc12835.roverruckus.subsystems.MecanumDrive;

public class DeadReckoningLocalizer implements Localizer {
    protected MecanumDrive drive;

    protected Vector2d estimatedPosition;
    private double[] lastRotations;

    public DeadReckoningLocalizer(MecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public Vector2d update() {
        double[] rotations = drive.getDriveMotorRotations();
        if (lastRotations != null) {
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Vector2d robotPoseDelta = DifferentialDrive.getPoseDelta(rotationDeltas).pos();
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(drive.getHeading());

            estimatedPosition = estimatedPosition.added(fieldPoseDelta);
        }
        lastRotations = rotations;
        return estimatedPosition;
    }

    @Override
    public void setEstimatedPosition(Vector2d position) {
        estimatedPosition = position;
    }
}
