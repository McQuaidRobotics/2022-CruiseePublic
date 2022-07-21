// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final DigitalOutput bitOne = new DigitalOutput(13);
  private final DigitalOutput bitTwo = new DigitalOutput(14);
  private final DigitalOutput bitThree = new DigitalOutput(15);
  private final DigitalOutput bitFour = new DigitalOutput(16);

  public void arduinoPattern(int num){
    bitOne.set((num & 0b0001) == 0b0001);
    bitTwo.set((num & 0b0010) == 0b0010);
    bitThree.set((num & 0b0100) == 0b0100);
    bitFour.set((num & 0b1000) == 0b1000);
  }
}
  