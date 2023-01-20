package com.arc852.lib.motors;

import java.util.HashMap;

public class ARCTalonManager {
  private static final HashMap<Integer, ARCTalon> motors = new HashMap<>();

  public static ARCTalonMotor createTalonMotor(int id) {
    if ((motors.containsKey(id) && motors.get(id).getState().equals(TalonState.MOTOR))
        || motors.get(id).getState().equals(TalonState.SHARED)) {
      throw new IllegalArgumentException("Motor already exists");
    }
    if (motors.containsKey(id) && motors.get(id).getState().equals(TalonState.ENCODER)) {
      motors.get(id).setMotorType(TalonState.SHARED);
    }
    ARCTalonMotor motor = new ARCTalonMotor(id);
    motors.put(id, motor);
    return motor;
  }

  public static ARCTalonEncoder createTalonEncoder(int id) {
    if ((motors.containsKey(id) && !motors.get(id).getState().equals(TalonState.ENCODER))
        || motors.get(id).getState().equals(TalonState.SHARED)) {
      throw new IllegalArgumentException("Encoder already exists");
    } // perhaps we should allow multiple encoders to be created
    // the idea is that with motors conflicting set() calls could be an issue, but with encoders
    // conflicting read
    // calls is not
    if (motors.containsKey(id) && motors.get(id).getState().equals(TalonState.MOTOR)) {
      motors.get(id).setMotorType(TalonState.SHARED);
      return new ARCTalonEncoder(motors.get(id).getMotor());
    }
    ARCTalonEncoder encoder = new ARCTalonEncoder(id);
    motors.put(id, encoder);
    return encoder;
  }
}
