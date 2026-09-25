// Cyncoder magnetic encoder - simulation control.
//
// Under simulation a Cyncoder has no board to listen to, so it
// reads its telemetry out of a WPILib SimDevice instead. That makes the
// values show up (and be editable by hand) in the Simulation GUI under
// "Other Devices > Cyncoder[<CAN ID>]"; this class is the
// programmatic way into the same values, for a physics sim or a unit
// test that needs to drive the encoder from code.
//
//   private final Cyncoder m_encoder = new Cyncoder(12);
//   private final CyncoderSim m_encoderSim = new CyncoderSim(m_encoder);
//   ...
//   @Override
//   public void simulationPeriodic() {
//     m_armSim.update(0.02);
//     m_encoderSim.setRawPosition(Units.radiansToDegrees(m_armSim.getAngleRads()));
//     m_encoderSim.setRawVelocity(Units.radiansToDegrees(m_armSim.getVelocityRadPerSec()));
//   }
//
// Everything set here is in RAW SENSOR terms - the numbers the firmware
// would have put on the wire, before the encoder's own inversion,
// offsets, gear ratio and wrapping. That is the point: feeding the
// simulation at the wire means the code-side transforms under test are
// the real ones, not ones the test quietly bypassed.

package com.cygnus.CygnusLib;

public class CyncoderSim {
  private final Cyncoder m_encoder;

  /**
   * Wraps an encoder for simulated control.
   *
   * @throws IllegalStateException if the encoder has no simulation
   *     backing - i.e. you are running on a real roboRIO, where the
   *     board on the CAN bus is the only thing allowed to set these
   *     values.
   */
  public CyncoderSim(Cyncoder encoder) {
    if (encoder.m_simDevice == null) {
      throw new IllegalStateException(
          "CyncoderSim needs a Cyncoder created in simulation; CAN ID "
              + encoder.getDeviceId()
              + " is talking to real hardware");
    }
    m_encoder = encoder;
  }

  /**
   * Sets the raw sensor angle in degrees, exactly as the firmware would
   * report it: continuous and unbounded, so keep counting past 360 for a
   * multi-turn mechanism instead of wrapping it yourself. Wrapping is
   * what {@link Cyncoder#getAbsolutePosition()} is for, and doing it
   * here would hide whether that works.
   */
  public void setRawPosition(double positionDegrees) {
    m_encoder.m_simRawPositionDeg.set(positionDegrees);
  }

  /** The raw sensor angle currently being simulated, in degrees. */
  public double getRawPosition() {
    return m_encoder.m_simRawPositionDeg.get();
  }

  /** Sets the raw sensor velocity in degrees per second, as the firmware would report it. */
  public void setRawVelocity(double velocityDegreesPerSecond) {
    m_encoder.m_simRawVelocityDegPerSec.set(velocityDegreesPerSecond);
  }

  /** The raw sensor velocity currently being simulated, in degrees per second. */
  public double getRawVelocity() {
    return m_encoder.m_simRawVelocityDegPerSec.get();
  }

  /**
   * Simulates the board being present on the bus or not - the switch
   * behind {@link Cyncoder#isConnected()}. Set it false to check
   * that a subsystem degrades sanely when a CAN wire comes loose mid
   * match. Defaults to true.
   */
  public void setConnected(boolean connected) {
    m_encoder.m_simConnected.set(connected);
  }

  /** Whether the simulated board is currently on the bus. */
  public boolean getConnected() {
    return m_encoder.m_simConnected.get();
  }

  /**
   * Simulates the magnet-health reading. {@link CyncoderMagnetHealth#UNKNOWN}
   * is not a firmware state and is rejected here - to simulate "no
   * health data", use {@link #setConnected(boolean)} with false, which is
   * what actually produces UNKNOWN on a real robot.
   *
   * @throws IllegalArgumentException if passed UNKNOWN
   */
  public void setMagnetHealth(CyncoderMagnetHealth health) {
    if (health == CyncoderMagnetHealth.UNKNOWN) {
      throw new IllegalArgumentException(
          "UNKNOWN is a no-data state, not something the board reports; use setConnected(false)");
    }
    m_encoder.m_simMagnetHealth.set(health.getValue());
  }

  /** The magnet health currently being simulated. */
  public CyncoderMagnetHealth getMagnetHealth() {
    return CyncoderMagnetHealth.fromValue(m_encoder.m_simMagnetHealth.get());
  }

  /**
   * Simulates the sensor's automatic-gain-control reading, 0-255.
   *
   * @throws IllegalArgumentException if outside 0-255
   */
  public void setAGC(int agc) {
    if (agc < 0 || agc > 255) {
      throw new IllegalArgumentException("AGC must be 0-255, got " + agc);
    }
    m_encoder.m_simAgc.set(agc);
  }

  /**
   * Simulates the sensor's field-strength magnitude, 0-4095.
   *
   * @throws IllegalArgumentException if outside 0-4095
   */
  public void setMagnitude(int magnitude) {
    if (magnitude < 0 || magnitude > 4095) {
      throw new IllegalArgumentException("magnitude must be 0-4095, got " + magnitude);
    }
    m_encoder.m_simMagnitude.set(magnitude);
  }
}
