// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveDrive;

public class CreateEventMap {

  HashMap<String, Command> eventMap = new HashMap<>();

  SwerveDrive m_drivebase;

    public CreateEventMap(SwerveDrive m_drivebase) {
    this.m_drivebase = m_drivebase;

  }

  public HashMap<String, Command> createMap(){
    return eventMap;
  }
}
