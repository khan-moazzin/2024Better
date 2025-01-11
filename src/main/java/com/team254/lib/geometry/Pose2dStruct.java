// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Pose2dStruct implements Struct<com.team254.lib.geometry.Pose2d> {
  @Override
  public Class<com.team254.lib.geometry.Pose2d> getTypeClass() {
    return com.team254.lib.geometry.Pose2d.class;
  }

  @Override
  public String getTypeName() {
    return "Pose2d+";
  }

  @Override
  public int getSize() {
    return Pose2d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Pose2d pose";
  }

  @Override
  public Struct<?>[] getNested() {
    return Pose2d.struct.getNested();
  }

  @Override
  public com.team254.lib.geometry.Pose2d unpack(ByteBuffer bb) {
    return new com.team254.lib.geometry.Pose2d(Pose2d.struct.unpack(bb));
  }

  @Override
  public void pack(ByteBuffer bb, com.team254.lib.geometry.Pose2d value) {
    Pose2d.struct.pack(bb, value.wpi());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
