// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.geometry;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dStruct implements Struct<com.team254.lib.geometry.Translation2d> {
  @Override
  public Class<com.team254.lib.geometry.Translation2d> getTypeClass() {
    return com.team254.lib.geometry.Translation2d.class;
  }

  @Override
  public String getTypeName() {
    return "Translation2d+";
  }

  @Override
  public int getSize() {
    return Translation2d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Translation2d translation";
  }

  @Override
  public com.team254.lib.geometry.Translation2d unpack(ByteBuffer bb) {
    return new com.team254.lib.geometry.Translation2d(Translation2d.struct.unpack(bb));
  }

  @Override
  public void pack(ByteBuffer bb, com.team254.lib.geometry.Translation2d value) {
    Translation2d.struct.pack(bb, value.wpi());
  }
  @Override
  public boolean isImmutable() {
    return true;
  }
}
