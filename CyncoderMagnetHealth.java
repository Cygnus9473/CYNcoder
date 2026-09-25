// Cyncoder - magnet health state.

package com.cygnus.CygnusLib;

/**
 * How well the magnet is seated over the sensor, as judged by the
 * firmware from the sensor's magnet status (magnet detected, field too
 * weak, field too strong). The numeric codes match the firmware's health
 * codes and backend/can_protocol.py exactly, so the same value means the
 * same thing in the firmware, the Tuner app and here.
 */
public enum CyncoderMagnetHealth {
  /** Magnet detected and in range - readings are trustworthy. */
  GOOD(0),

  /** Magnet detected but too weak (too far from the chip) - readings may drift or jump. */
  FAR(1),

  /** No magnet detected, or it is too strong/too close - readings are not trustworthy. */
  ERROR(2),

  /**
   * The board cannot read its sensor: it has stopped answering on the
   * board's internal bus. The board stops sending position frames, so
   * {@link Cyncoder#isConnected()} turns false too. Check the sensor's
   * connection on the board. Sent by firmware protocol version 3 and later.
   */
  SENSOR_FAULT(3),

  /**
   * No magnet-health frame has been received yet. Not a firmware state:
   * this is what the library reports before the first frame arrives, or
   * after the device has dropped off the bus.
   */
  UNKNOWN(-1);

  private final int value;

  CyncoderMagnetHealth(int value) {
    this.value = value;
  }

  /** The on-the-wire code, or -1 for {@link #UNKNOWN}. */
  public int getValue() {
    return value;
  }

  /**
   * Maps a health code to its enum constant: 0-3 to the firmware states, -1
   * to {@link #UNKNOWN} (so {@code fromValue(h.getValue()) == h} for every
   * constant), and any other value to {@link #ERROR}.
   */
  public static CyncoderMagnetHealth fromValue(int value) {
    switch (value) {
      case 0:
        return GOOD;
      case 1:
        return FAR;
      case 3:
        return SENSOR_FAULT;
      case 2:
        return ERROR;
      case -1:
        return UNKNOWN;
      default:
        // The firmware only ever sends 0-3. A value outside that set
        // means a corrupted frame or a firmware/library version mismatch
        // - either way, "don't trust the reading" is the safe reading.
        return ERROR;
    }
  }
}
