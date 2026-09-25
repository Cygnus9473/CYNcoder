// Cyncoder - outcome of the most recent command sent to a board.

package com.cygnus.CygnusLib;

/**
 * What became of the last {@link Cyncoder#setZero()}, {@link Cyncoder#blink()}
 * or {@link Cyncoder#factoryDefault()} call, as reported by
 * {@link Cyncoder#getCommandStatus()}.
 *
 * <p>Firmware from protocol version 3 answers every command addressed to it
 * with an ACK frame, so the outcome is known rather than assumed. Older
 * firmware sends no ACK; its commands end as {@link #UNCONFIRMED}.
 */
public enum CyncoderCommandStatus {
  /** No command has been sent by this object yet. */
  NONE,

  /**
   * The command was never put on the bus: the board's UID is not known yet
   * (it is broadcast every 500 ms), or the transmit queue was full.
   */
  NOT_SENT,

  /** Sent; waiting for the board's ACK. */
  PENDING,

  /** The board applied the command, and saved it to Flash where it persists. */
  APPLIED,

  /**
   * The board applied the command, but writing Flash failed: the change is
   * live now and will be lost at the next power cycle.
   */
  APPLIED_NOT_SAVED,

  /** The board had not taken a sensor sample yet, so it changed nothing. Retry. */
  REJECTED_NOT_READY,

  /** The board rejected an argument (a Set Config CAN ID outside 1-62). */
  REJECTED_INVALID_ARGUMENT,

  /** Set Config: the board could not restart its CAN controller on the new ID. */
  CAN_ID_CHANGE_FAILED,

  /**
   * No ACK arrived in time. The frame may have been lost, the board may be
   * off the bus, or a different board may now hold this CAN ID (a board
   * never answers a command addressed to another board's UID).
   */
  TIMED_OUT,

  /**
   * Sent to firmware that predates command ACKs (protocol version below 3).
   * The command may well have worked; confirm by reading telemetry.
   */
  UNCONFIRMED,

  /** The board answered with a result code this library does not know. */
  UNKNOWN_RESULT;

  /** Whether the outcome is final, i.e. anything but {@link #PENDING}. */
  public boolean isDone() {
    return this != PENDING;
  }

  /** Whether the board confirmed the command fully applied. */
  public boolean isSuccess() {
    return this == APPLIED;
  }

  /** Maps a COMMAND_ACK result code to a status. */
  public static CyncoderCommandStatus fromAckResult(int result) {
    switch (result) {
      case CyncoderCANProtocol.ACK_OK:
        return APPLIED;
      case CyncoderCANProtocol.ACK_FLASH_WRITE_FAILED:
        return APPLIED_NOT_SAVED;
      case CyncoderCANProtocol.ACK_INVALID_ARGUMENT:
        return REJECTED_INVALID_ARGUMENT;
      case CyncoderCANProtocol.ACK_CAN_ID_CHANGE_FAILED:
        return CAN_ID_CHANGE_FAILED;
      case CyncoderCANProtocol.ACK_NOT_READY:
        return REJECTED_NOT_READY;
      default:
        return UNKNOWN_RESULT;
    }
  }
}
