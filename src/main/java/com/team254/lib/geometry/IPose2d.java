package com.team254.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    Pose2d getPose();  // <- fix this line
    S transformBy(Pose2d transform);
    S mirror();
    S mirrorAboutX(double xValue);
    S mirrorAboutY(double yValue);
}
