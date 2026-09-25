// Cyncoder - CAN wire protocol constants and payload codecs.
//
// This is the robot-code mirror of backend/can_protocol.py and the
// firmware's stm32_taslak/Core/Inc/can_handler.h. All three must agree
// bit-for-bit; if you change one, change all three.
//
// Arbitration ID layout (WPILib's extended 29-bit CAN convention):
//
//   bit 28-24  Device Type    (5 bits)  = DEVICE_TYPE
//   bit 23-16  Manufacturer   (8 bits)  = MANUFACTURER
//   bit 15-10  API Class      (6 bits)
//   bit  9-6   API Index      (4 bits)
//   bit  5-0   Device Number  (6 bits)  = the CAN ID (1-62) you set in the Tuner
//
// Cyncoder talks to the board through WPILib's stock
// edu.wpi.first.wpilibj.CAN class, which takes the manufacturer and device
// type as arguments - no JNI of its own is needed. WPILib packs
// API Class and API Index into one 10-bit "API ID" argument, so every
// frame below is expressed that way: apiId = (apiClass << 4) | apiIndex.

package com.cygnus.CygnusLib;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public final class CyncoderCANProtocol {
  private CyncoderCANProtocol() {}

  /**
   * FRC device type of every Cyncoder frame (bits 28-24). Currently 10,
   * "Miscellaneous". Must equal CAN_DEVICE_TYPE in the firmware and
   * DEVICE_TYPE in backend/can_protocol.py.
   */
  public static final int DEVICE_TYPE = 10;

  /**
   * FRC manufacturer code of every Cyncoder frame (bits 23-16). Currently
   * 8, "Team Use", pending an assigned manufacturer code. Must equal
   * CAN_MANUFACTURER in the firmware and MANUFACTURER in
   * backend/can_protocol.py; changing it is a protocol change for all three.
   */
  public static final int MANUFACTURER = 8;

  /** Lowest / highest CAN ID the 6-bit device-number field can address (0 is broadcast). */
  public static final int MIN_DEVICE_ID = 1;
  public static final int MAX_DEVICE_ID = 62;

  // ---- API Class / Index, as in can_handler.h -------------------------
  public static final int API_CLASS_TELEMETRY = 0;
  public static final int API_INDEX_POSITION_VELOCITY = 0;
  public static final int API_INDEX_MAGNET_HEALTH = 1;
  public static final int API_INDEX_DEVICE_UID = 2;
  /** Reply to a command (protocol v3). See {@link #unpackCommandAck(byte[])}. */
  public static final int API_INDEX_COMMAND_ACK = 3;

  public static final int API_CLASS_COMMAND = 1;
  public static final int API_INDEX_SET_ZERO = 0;
  public static final int API_INDEX_SET_CONFIG = 1;
  public static final int API_INDEX_FACTORY_DEFAULT = 2;
  public static final int API_INDEX_BLINK = 3;

  // ---- Packed 10-bit API IDs, ready for edu.wpi.first.wpilibj.CAN -----
  public static final int API_ID_POSITION_VELOCITY = apiId(API_CLASS_TELEMETRY, API_INDEX_POSITION_VELOCITY);
  public static final int API_ID_MAGNET_HEALTH = apiId(API_CLASS_TELEMETRY, API_INDEX_MAGNET_HEALTH);
  public static final int API_ID_DEVICE_UID = apiId(API_CLASS_TELEMETRY, API_INDEX_DEVICE_UID);
  public static final int API_ID_COMMAND_ACK = apiId(API_CLASS_TELEMETRY, API_INDEX_COMMAND_ACK);

  public static final int API_ID_SET_ZERO = apiId(API_CLASS_COMMAND, API_INDEX_SET_ZERO);
  public static final int API_ID_SET_CONFIG = apiId(API_CLASS_COMMAND, API_INDEX_SET_CONFIG);
  public static final int API_ID_FACTORY_DEFAULT = apiId(API_CLASS_COMMAND, API_INDEX_FACTORY_DEFAULT);
  public static final int API_ID_BLINK = apiId(API_CLASS_COMMAND, API_INDEX_BLINK);

  // ---- Firmware transmit periods (stm32_taslak/Core/Src/main.c) -------
  /** Position/velocity + magnet health are broadcast at 100 Hz. */
  public static final int TELEMETRY_PERIOD_MS = 10;
  /** The device-identity (UID) frame is broadcast at 2 Hz. */
  public static final int UID_PERIOD_MS = 500;

  // ---- Protocol version (magnet-health frame byte 6) -------------------
  /**
   * The protocol version this library was written against. Version 2 added
   * bytes 4-7 of the magnet-health frame (exact absolute ticks, version,
   * bus-off count); version 3 added command ACKs. Firmware before version
   * 2 zero-padded those bytes, so a version byte of 0 identifies it.
   */
  public static final int PROTOCOL_VERSION = 3;

  /** First protocol version whose firmware answers commands with an ACK. */
  public static final int FIRST_VERSION_WITH_COMMAND_ACK = 3;

  // ---- Command ACK result codes (COMMAND_ACK byte 1) ---------------------
  /** Applied, and saved to Flash where the command persists. */
  public static final int ACK_OK = 0;
  /** Applied now, but the Flash write failed: it will not survive a reboot. */
  public static final int ACK_FLASH_WRITE_FAILED = 1;
  /** SET_CONFIG with an out-of-range CAN ID: ID kept, direction invert applied. */
  public static final int ACK_INVALID_ARGUMENT = 2;
  /** SET_CONFIG: the board could not restart its CAN controller on the new ID. */
  public static final int ACK_CAN_ID_CHANGE_FAILED = 3;
  /** The board had no sensor sample yet; nothing was changed. */
  public static final int ACK_NOT_READY = 4;

  /**
   * A decoded COMMAND_ACK frame.
   *
   * @param commandIndex the API index of the command answered, e.g. {@link #API_INDEX_SET_ZERO}
   * @param result one of the {@code ACK_*} codes
   * @param sequence the sequence number the command carried, 0 if it had none
   * @param uid the UID of the board that answered
   */
  public record CommandAck(int commandIndex, int result, int sequence, long uid) {}

  /** Sensor counts per revolution (12-bit). */
  public static final int TICKS_PER_REVOLUTION = 4096;

  /** Converts single-turn sensor ticks (0-4095) to degrees. */
  public static double ticksToDegrees(int ticks) {
    return ticks * (360.0 / TICKS_PER_REVOLUTION);
  }

  /** Combines an API Class and API Index into the 10-bit API ID WPILib's CAN class wants. */
  public static int apiId(int apiClass, int apiIndex) {
    return ((apiClass & 0x3F) << 4) | (apiIndex & 0x0F);
  }

  // ---- Payload codecs -------------------------------------------------
  //
  // Every multi-byte field is little-endian, matching the STM32's native
  // byte order (the firmware memcpy's its floats straight into the frame).

  private static ByteBuffer le(byte[] data, int length) {
    return ByteBuffer.wrap(data, 0, length).order(ByteOrder.LITTLE_ENDIAN);
  }

  /**
   * Decodes the 8-byte "Status 1" telemetry frame into
   * {@code {positionDegrees, velocityDegreesPerSecond}}.
   *
   * <p>Note that position is NOT wrapped to 0-360: the firmware tracks a
   * continuous, unbounded multi-turn angle (it can be negative or exceed
   * 360), already with the device's persisted zero offset and
   * direction-invert applied. Wrapping is the robot code's choice, which
   * is what Cyncoder's position-wrapping settings are for.
   */
  public static double[] unpackPositionVelocity(byte[] data) {
    ByteBuffer buffer = le(data, 8);
    float positionDeg = buffer.getFloat();
    float velocityDegPerSec = buffer.getFloat();
    return new double[] {positionDeg, velocityDegPerSec};
  }

  /** Decodes the magnet-health frame: health code, AGC (0-255), field-strength magnitude (0-4095). */
  public static int[] unpackMagnetHealth(byte[] data) {
    ByteBuffer buffer = le(data, 4);
    int healthCode = buffer.get() & 0xFF;
    int agc = buffer.get() & 0xFF;
    int magnitude = buffer.getShort() & 0x0FFF;
    return new int[] {healthCode, agc, magnitude};
  }

  /**
   * Decodes the protocol-v2 extension in bytes 4-7 of the magnet-health
   * frame into {@code {absoluteTicks, protocolVersion, busOffCount}}.
   *
   * <p>absoluteTicks is the single-turn angle, 0-4095, with the board's
   * stored zero and direction invert applied. Unlike the float position in
   * the Status 1 frame, it is exact regardless of how many turns the
   * mechanism has made.
   *
   * @param length the frame's actual data length
   * @return the decoded fields, or null for a frame from firmware that
   *     predates the extension (shorter than 8 bytes, or version byte 0)
   */
  public static int[] unpackHealthExtension(byte[] data, int length) {
    if (length < 8) {
      return null;
    }
    ByteBuffer buffer = le(data, 8);
    int absoluteTicks = buffer.getShort(4) & (TICKS_PER_REVOLUTION - 1);
    int version = buffer.get(6) & 0xFF;
    int busOffCount = buffer.get(7) & 0xFF;
    if (version == 0) {
      return null;
    }
    return new int[] {absoluteTicks, version, busOffCount};
  }

  /** Decodes the 4-byte device-identity frame, as an unsigned 32-bit value widened to a long. */
  public static long unpackDeviceUid(byte[] data) {
    return le(data, 4).getInt() & 0xFFFFFFFFL;
  }

  /**
   * Builds the payload shared by SET_ZERO, FACTORY_DEFAULT and BLINK: the
   * 4-byte UID of the device the command is meant for.
   *
   * <p>The firmware ignores any command whose target UID isn't its own.
   * That guard exists because two boards can legitimately answer to the
   * same CAN ID (e.g. both fresh from the bench at ID 15), and without it
   * a command addressed to that ID would be obeyed by both of them - see
   * the CAN_API_CLASS_COMMAND comment in can_handler.h.
   */
  public static byte[] packTargetUid(long targetUid) {
    return ByteBuffer.allocate(4)
        .order(ByteOrder.LITTLE_ENDIAN)
        .putInt((int) targetUid)
        .array();
  }

  /**
   * Like {@link #packTargetUid(long)}, plus the protocol-v3 sequence byte
   * the board echoes in its ACK. Firmware before v3 ignores the extra byte.
   *
   * @param sequence 1-255; 0 is reserved for "no sequence"
   */
  public static byte[] packTargetUid(long targetUid, int sequence) {
    return ByteBuffer.allocate(5)
        .order(ByteOrder.LITTLE_ENDIAN)
        .putInt((int) targetUid)
        .put((byte) sequence)
        .array();
  }

  /** Decodes an 8-byte COMMAND_ACK frame. */
  public static CommandAck unpackCommandAck(byte[] data) {
    ByteBuffer buffer = le(data, 8);
    int commandIndex = buffer.get(0) & 0xFF;
    int result = buffer.get(1) & 0xFF;
    int sequence = buffer.get(2) & 0xFF;
    long uid = buffer.getInt(4) & 0xFFFFFFFFL;
    return new CommandAck(commandIndex, result, sequence, uid);
  }
}
