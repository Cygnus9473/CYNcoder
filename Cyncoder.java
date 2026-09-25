// Cyncoder magnetic encoder - robot-code sensor class.
//
// Talks to the STM32 board over the robot's CAN bus using WPILib's stock
// edu.wpi.first.wpilibj.CAN class. No JNI, no vendordep, no extra
// libraries: our arbitration IDs already use WPILib's "Team Use"
// manufacturer (8) and "Miscellaneous" device type (10), which is
// exactly what that class addresses by default.
//
//   private final Cyncoder m_turnEncoder = new Cyncoder(12);
//   ...
//   double angleDeg = m_turnEncoder.getAbsolutePosition();
//
// THIS IS NOT CyncoderBridge. CyncoderBridge relays raw CAN frames to the
// Cygnus Tuner desktop app over Wi-Fi so you can configure the boards;
// this class is what your actual robot code reads the sensor with. They
// are independent - run both, or just this one.
//
// UNITS: degrees and degrees per second, everywhere, matching the
// firmware and the Tuner app's own display so the numbers you see on
// the dashboard are the numbers you see in the Tuner. Divide by 360 if
// you need rotations.
//
// ON-DEVICE vs. IN-CODE SETTINGS: the board itself persists a zero point
// and a direction-invert flag in Flash (set from the Tuner, or from
// setZero() below). Everything else this class offers - setInverted(),
// setMagnetOffset(), setPositionOffset(), setSensorToMechanismRatio(),
// position wrapping - is applied here in robot code on top of whatever
// the device reports, and is forgotten at power-off. That split is
// deliberate: the mechanical "which way is up" calibration belongs to
// the board, the code-side transforms belong to the subsystem using it.

package com.cygnus.CygnusLib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;
import java.util.concurrent.ThreadLocalRandom;

public class Cyncoder implements Sendable, AutoCloseable {
  /** Which single-turn range {@link #getAbsolutePosition()} folds the sensor angle into. */
  public enum AbsoluteRange {
    /** 0 to 360 degrees - the default, and what the Tuner app displays. */
    ZERO_TO_360(0.0, 360.0),
    /** -180 to +180 degrees - usually what you want for a swerve steering angle. */
    PLUS_MINUS_180(-180.0, 180.0);

    private final double min;
    private final double max;

    AbsoluteRange(double min, double max) {
      this.min = min;
      this.max = max;
    }
  }

  /**
   * All position and velocity readings from one telemetry frame, taken
   * together. Reading the getters one by one can straddle a frame
   * boundary; a snapshot cannot. Units are degrees and degrees per second.
   *
   * @param absolutePosition same as {@link Cyncoder#getAbsolutePosition()}
   * @param position same as {@link Cyncoder#getPosition()}
   * @param absoluteVelocity same as {@link Cyncoder#getAbsoluteVelocity()}
   * @param velocity same as {@link Cyncoder#getVelocity()}
   * @param connected same as {@link Cyncoder#isConnected()}
   * @param timestampSeconds same as {@link Cyncoder#getLastUpdateTimestamp()}
   */
  public record Snapshot(
      double absolutePosition,
      double position,
      double absoluteVelocity,
      double velocity,
      boolean connected,
      double timestampSeconds) {
    /** {@link #absolutePosition()} as a WPILib unit type. */
    public Angle absolutePositionMeasure() {
      return Degrees.of(absolutePosition);
    }

    /** {@link #position()} as a WPILib unit type. */
    public Angle positionMeasure() {
      return Degrees.of(position);
    }

    /** {@link #absoluteVelocity()} as a WPILib unit type. */
    public AngularVelocity absoluteVelocityMeasure() {
      return DegreesPerSecond.of(absoluteVelocity);
    }

    /** {@link #velocity()} as a WPILib unit type. */
    public AngularVelocity velocityMeasure() {
      return DegreesPerSecond.of(velocity);
    }
  }

  /**
   * How long a telemetry frame stays "live" before {@link #isConnected()}
   * gives up on it. The firmware broadcasts at 100 Hz, so 100 ms is ten
   * missed frames in a row - late enough not to flicker on a single
   * dropped frame, early enough that a yanked CAN wire shows up within
   * about two robot loops.
   */
  private static final double DEFAULT_CONNECTION_TIMEOUT_SECONDS = 0.1;

  /**
   * How long to wait for a command ACK. Commands that write Flash are
   * answered after the write (tens of milliseconds); this leaves ample
   * margin on a busy bus.
   */
  private static final long COMMAND_ACK_TIMEOUT_MS = 500;

  private final int m_deviceId;

  /** The real CAN link. Null in simulation, where {@link #m_simDevice} stands in for it. */
  private final CAN m_can;

  // Reused across reads instead of allocated per call: these getters run
  // every loop for every encoder on the robot, and CANData carries a
  // fixed 8-byte array anyway. Guarded by this object's monitor.
  private final CANData m_positionData = new CANData();
  private final CANData m_healthData = new CANData();
  private final CANData m_uidData = new CANData();
  private final CANData m_ackData = new CANData();

  // ---- Simulation backing store (null on a real robot) ---------------
  final SimDevice m_simDevice;
  SimDouble m_simRawPositionDeg;
  SimDouble m_simRawVelocityDegPerSec;
  SimInt m_simMagnetHealth;
  SimInt m_simAgc;
  SimInt m_simMagnitude;
  SimBoolean m_simConnected;

  // ---- Code-side configuration ---------------------------------------
  private boolean m_inverted;
  private double m_magnetOffsetDeg;
  private double m_positionOffsetDeg;
  private double m_sensorToMechanismRatio = 1.0;
  private AbsoluteRange m_absoluteRange = AbsoluteRange.ZERO_TO_360;
  private boolean m_positionWrappingEnabled;
  private double m_positionWrapMinDeg;
  private double m_positionWrapMaxDeg = 360.0;
  private long m_connectionTimeoutMs = (long) (DEFAULT_CONNECTION_TIMEOUT_SECONDS * 1000.0);

  // ---- Last-known telemetry ------------------------------------------
  private double m_rawPositionDeg;
  private double m_rawVelocityDegPerSec;
  private boolean m_positionFresh;
  // CAN frame timestamps are on the HAL's CAN timebase (a 32-bit millisecond
  // counter, CLOCK_MONOTONIC on the roboRIO), not the FPGA clock. Ages are
  // always computed against CAN.getTimestampBaseTime(), with 32-bit wrap.
  private long m_positionFrameCanMs = -1;
  private double m_lastPositionFpgaSeconds = -1.0;

  private CyncoderMagnetHealth m_health = CyncoderMagnetHealth.UNKNOWN;
  private int m_agc;
  private int m_magnitude;

  // From the protocol-v2 extension of the health frame; -1 / 0 until a
  // current v2 frame has been seen (older firmware never sends them).
  private int m_absoluteTicks = -1;
  private int m_protocolVersion;
  private int m_busOffCount = -1;
  private boolean m_healthFrameSeen;
  private long m_healthFrameCanMs = -1;

  // ---- Last command sent, and its ACK ---------------------------------
  // Random start: an ACK left over from a previous robot-program run can
  // never carry the sequence the first new command uses by accident.
  private int m_nextSequence = ThreadLocalRandom.current().nextInt(1, 256);
  private int m_pendingCommandIndex = -1;
  private int m_pendingSequence;
  private long m_pendingUid;
  private long m_pendingSentCanMs;
  private CyncoderCommandStatus m_commandStatus = CyncoderCommandStatus.NONE;

  private long m_deviceUid = -1;

  // ---- Driver-station alerts -------------------------------------------
  // WPILib's Alert is not thread-safe: all Alerts share one unsynchronized
  // set per group, which the main thread iterates while publishing. The
  // alerts are therefore only changed from the thread that created this
  // encoder (normally the main robot thread); reads from other threads just
  // mark them for update on the next read from that thread.
  private final Alert m_disconnectedAlert;
  private final Alert m_magnetAlert;
  private final Thread m_ownerThread;
  private boolean m_alertsEnabled = true;

  private boolean m_closed;

  /**
   * Creates an encoder on the given CAN ID (1-62), the same number shown
   * next to the board in the Cygnus Tuner, with the default settings.
   *
   * @param deviceId the board's CAN ID, 1-62
   * @throws IllegalArgumentException if the CAN ID is outside 1-62 (0 is
   *     the broadcast address and the field is only 6 bits wide, so
   *     nothing outside that range is addressable).
   * @throws IllegalStateException in simulation, if another open Cyncoder
   *     already uses this CAN ID.
   */
  public Cyncoder(int deviceId) {
    this(deviceId, new CyncoderConfig(), true);
  }

  /**
   * Creates an encoder and applies {@code config} to it - the same as
   * {@link #Cyncoder(int)} followed by {@link #applyConfig(CyncoderConfig)}.
   *
   * @param deviceId the board's CAN ID, 1-62
   * @param config settings to apply
   * @throws IllegalArgumentException if the CAN ID is outside 1-62
   * @throws NullPointerException if {@code config} is null (checked before
   *     anything is allocated)
   * @throws IllegalStateException in simulation, if another open Cyncoder
   *     already uses this CAN ID.
   */
  public Cyncoder(int deviceId, CyncoderConfig config) {
    this(deviceId, Objects.requireNonNull(config, "config"), true);
  }

  // Every argument is checked before any resource is allocated, so a
  // constructor that throws leaves nothing registered behind.
  private Cyncoder(int deviceId, CyncoderConfig config, boolean validated) {
    if (deviceId < CyncoderCANProtocol.MIN_DEVICE_ID || deviceId > CyncoderCANProtocol.MAX_DEVICE_ID) {
      throw new IllegalArgumentException(
          "Cyncoder CAN ID must be "
              + CyncoderCANProtocol.MIN_DEVICE_ID
              + "-"
              + CyncoderCANProtocol.MAX_DEVICE_ID
              + ", got "
              + deviceId);
    }
    m_deviceId = deviceId;
    m_ownerThread = Thread.currentThread();

    if (RobotBase.isSimulation()) {
      // Don't open a CAN session at all under simulation - there's no
      // board on the other end, and going through SimDevice instead puts
      // these values in the Simulation GUI where CyncoderSim (and a
      // human poking at the GUI) can drive them.
      m_can = null;
      m_simDevice = SimDevice.create("Cyncoder", deviceId);
      if (m_simDevice == null) {
        // A second encoder on the same ID would otherwise be silently dead
        // in simulation while working on a real robot.
        throw new IllegalStateException(
            "Another open Cyncoder already uses CAN ID "
                + deviceId
                + " in simulation; close() it before creating a new one");
      }
      m_simRawPositionDeg =
          m_simDevice.createDouble("rawPositionDeg", SimDevice.Direction.kInput, 0.0);
      m_simRawVelocityDegPerSec =
          m_simDevice.createDouble("rawVelocityDegPerSec", SimDevice.Direction.kInput, 0.0);
      m_simMagnetHealth =
          m_simDevice.createInt(
              "magnetHealth", SimDevice.Direction.kInput, CyncoderMagnetHealth.GOOD.getValue());
      m_simAgc = m_simDevice.createInt("agc", SimDevice.Direction.kInput, 128);
      m_simMagnitude = m_simDevice.createInt("magnitude", SimDevice.Direction.kInput, 2048);
      m_simConnected = m_simDevice.createBoolean("connected", SimDevice.Direction.kInput, true);
    } else {
      m_can =
          new CAN(
              deviceId,
              CyncoderCANProtocol.MANUFACTURER,
              CyncoderCANProtocol.DEVICE_TYPE);
      m_simDevice = null;
    }

    m_disconnectedAlert = new Alert("Cyncoder " + deviceId + " disconnected", AlertType.kError);
    m_magnetAlert = new Alert("Cyncoder " + deviceId + " magnet fault", AlertType.kWarning);

    SendableRegistry.addLW(this, "Cyncoder", deviceId);
    applyConfig(config);
  }

  /**
   * Applies every setting in {@code config} at once. The config is always
   * valid (its {@code with} methods reject bad values), so this cannot fail
   * partway, and no reading can observe a mix of old and new settings.
   *
   * <p>Does not touch the position offset: homing done with
   * {@link #setPosition(double)} survives a config change.
   */
  public synchronized void applyConfig(CyncoderConfig config) {
    m_inverted = config.getInverted();
    m_magnetOffsetDeg = config.getMagnetOffset();
    m_sensorToMechanismRatio = config.getSensorToMechanismRatio();
    m_absoluteRange = config.getAbsoluteRange();
    m_positionWrappingEnabled = config.isPositionWrappingEnabled();
    m_positionWrapMinDeg = config.getPositionWrappingMin();
    m_positionWrapMaxDeg = config.getPositionWrappingMax();
    m_connectionTimeoutMs = Math.round(config.getConnectionTimeout() * 1000.0);
    setAlertsEnabled(config.getAlertsEnabled());
  }

  /**
   * The encoder's current code-side settings, as a new config. Changing
   * the returned object has no effect until it is applied.
   */
  public synchronized CyncoderConfig getConfig() {
    return new CyncoderConfig()
        .withInverted(m_inverted)
        .withMagnetOffset(m_magnetOffsetDeg)
        .withSensorToMechanismRatio(m_sensorToMechanismRatio)
        .withAbsoluteRange(m_absoluteRange)
        .withPositionWrappingEnabled(m_positionWrappingEnabled)
        .withPositionWrappingRange(m_positionWrapMinDeg, m_positionWrapMaxDeg)
        .withConnectionTimeout(getConnectionTimeout())
        .withAlertsEnabled(m_alertsEnabled);
  }

  /** The CAN ID this object was constructed with. */
  public int getDeviceId() {
    return m_deviceId;
  }

  // ====================================================================
  //  Position and velocity
  // ====================================================================

  /**
   * The single-turn sensor angle, folded into the configured
   * {@link AbsoluteRange} (0-360 degrees by default).
   *
   * <p>This is the reading that survives a power cycle: it comes straight
   * from the magnet's physical orientation plus the zero point stored in
   * the board's Flash, so it is the same number after a reboot with the
   * mechanism untouched. Use it to seed a relative encoder or to home a
   * swerve module at startup.
   *
   * <p>{@link #setSensorToMechanismRatio(double)} and
   * {@link #setPositionOffset(double)} deliberately do NOT affect this
   * value - it describes the sensor shaft itself, not the mechanism
   * behind a gearbox. {@link #setInverted(boolean)} and
   * {@link #setMagnetOffset(double)} do, since those describe the sensor.
   *
   * <p>With firmware protocol version 2 or later, this comes from the exact
   * single-turn tick count the board sends, so it stays exact however many
   * turns the mechanism has made. Older firmware only sends the multi-turn
   * float position, and the angle is derived from that.
   *
   * @return degrees in [0, 360) or [-180, 180), per the configured range
   */
  public synchronized double getAbsolutePosition() {
    refreshPositionVelocity();
    refreshMagnetHealth();
    return absolutePositionDegrees();
  }

  /**
   * The mechanism position: continuous and multi-turn by default, so it
   * keeps counting past 360 degrees and can go negative - the firmware
   * tracks wrap-around for you, which is the whole point of the board.
   *
   * <p>Scaled by {@link #setSensorToMechanismRatio(double)}, shifted by
   * {@link #setPositionOffset(double)} / {@link #setPosition(double)},
   * and folded into a range only if you asked for that with
   * {@link #setPositionWrappingEnabled(boolean)}.
   *
   * @return degrees
   */
  public synchronized double getPosition() {
    refreshPositionVelocity();
    return positionDegrees();
  }

  /**
   * Mechanism velocity - the sensor's velocity divided by
   * {@link #setSensorToMechanismRatio(double)}, negated if
   * {@link #setInverted(boolean)} is set.
   *
   * @return degrees per second
   */
  public synchronized double getVelocity() {
    refreshPositionVelocity();
    return velocityDegrees();
  }

  /**
   * The raw sensor-shaft velocity, with no gear ratio applied - the
   * velocity counterpart of {@link #getAbsolutePosition()}, in the same
   * frame of reference. Offsets never affect a velocity, so
   * {@link #setInverted(boolean)} is the only setting that changes this.
   *
   * <p>Position wrapping does not affect it either: the firmware derives
   * velocity from raw sensor deltas before any wrapping, so a mechanism
   * rolling through the 360-to-0 seam reports a smooth speed here rather
   * than a one-sample spike.
   *
   * @return degrees per second at the sensor shaft
   */
  public synchronized double getAbsoluteVelocity() {
    refreshPositionVelocity();
    return sensorVelocityDegrees();
  }

  /**
   * Shifts {@link #getPosition()} so that it reads {@code positionDegrees}
   * right now, by adjusting the code-side position offset.
   *
   * <p>Purely local and forgotten at power-off - it does not write to the
   * board's Flash and does not move the absolute zero point. Use
   * {@link #setZero()} for that.
   *
   * @return false, and changes nothing, if the encoder is not connected:
   *     the offset would be computed against a stale or never-received
   *     reading. Retry once {@link #isConnected()} is true.
   *
   * @throws IllegalArgumentException if {@code positionDegrees} is NaN or infinite
   */
  public synchronized boolean setPosition(double positionDegrees) {
    CyncoderConfig.checkFinite("position", positionDegrees);
    refreshPositionVelocity();
    if (!m_positionFresh) {
      return false;
    }
    m_positionOffsetDeg = positionDegrees - sensorDegrees() / m_sensorToMechanismRatio;
    return true;
  }

  /**
   * All four position/velocity readings plus connection state, computed
   * from one telemetry frame. Prefer this when using position and
   * velocity together, e.g. as a feedforward/feedback pair.
   */
  public synchronized Snapshot getSnapshot() {
    refreshPositionVelocity();
    refreshMagnetHealth();
    return new Snapshot(
        absolutePositionDegrees(),
        positionDegrees(),
        sensorVelocityDegrees(),
        velocityDegrees(),
        m_positionFresh,
        timestampSeconds());
  }

  // ====================================================================
  //  Code-side configuration
  // ====================================================================

  /**
   * Flips the sign of every position and velocity this class reports.
   *
   * <p>Independent of, and applied on top of, the direction-invert flag
   * stored in the board's Flash by the Tuner. If the Tuner already has
   * the board turning the right way, leave this alone.
   */
  public synchronized void setInverted(boolean inverted) {
    m_inverted = inverted;
  }

  /** Whether the code-side sign flip from {@link #setInverted(boolean)} is on. */
  public synchronized boolean getInverted() {
    return m_inverted;
  }

  /**
   * A code-side angular offset added to the sensor angle, in degrees,
   * before anything else - so it moves {@link #getAbsolutePosition()} and
   * {@link #getPosition()} together.
   *
   * <p>This is the "my magnet is mounted 37 degrees off" correction. It
   * lives in code, so it survives a board swap but not a rewrite of your
   * constants; a calibration you want burned into the board itself
   * should go through {@link #setZero()} or the Tuner instead.
   *
   * @throws IllegalArgumentException if {@code offsetDegrees} is NaN or infinite
   */
  public synchronized void setMagnetOffset(double offsetDegrees) {
    CyncoderConfig.checkFinite("magnet offset", offsetDegrees);
    m_magnetOffsetDeg = offsetDegrees;
  }

  /**
   * @return the offset set by {@link #setMagnetOffset(double)}, in degrees
   */
  public synchronized double getMagnetOffset() {
    return m_magnetOffsetDeg;
  }

  /**
   * A code-side offset added to {@link #getPosition()} only, in mechanism
   * degrees - i.e. applied after the gear ratio, unlike
   * {@link #setMagnetOffset(double)}.
   *
   * @throws IllegalArgumentException if {@code offsetDegrees} is NaN or infinite
   */
  public synchronized void setPositionOffset(double offsetDegrees) {
    CyncoderConfig.checkFinite("position offset", offsetDegrees);
    m_positionOffsetDeg = offsetDegrees;
  }

  /**
   * @return the offset set by {@link #setPositionOffset(double)}, in degrees
   */
  public synchronized double getPositionOffset() {
    return m_positionOffsetDeg;
  }

  /**
   * Sensor rotations per mechanism rotation - the reduction between the
   * magnet and the thing you actually care about. With a 12.8:1 gearbox
   * between the encoder and the arm, pass 12.8 and
   * {@link #getPosition()} starts reading arm degrees.
   *
   * <p>Only affects {@link #getPosition()} and {@link #getVelocity()};
   * the absolute readings stay in sensor-shaft terms, because a
   * single-turn absolute angle stops being absolute once you divide it
   * by a ratio greater than 1.
   *
   * @throws IllegalArgumentException if the ratio is zero, negative or not finite -
   *     use {@link #setInverted(boolean)} to reverse direction, not a
   *     negative ratio, so the sign lives in exactly one place.
   */
  public synchronized void setSensorToMechanismRatio(double ratio) {
    CyncoderConfig.checkSensorToMechanismRatio(ratio);
    m_sensorToMechanismRatio = ratio;
  }

  /**
   * @return the ratio set by {@link #setSensorToMechanismRatio(double)}
   */
  public synchronized double getSensorToMechanismRatio() {
    return m_sensorToMechanismRatio;
  }

  /**
   * Chooses the range {@link #getAbsolutePosition()} folds into. Defaults to 0-360.
   *
   * @param range the range
   * @throws NullPointerException if {@code range} is null
   */
  public synchronized void setAbsoluteRange(AbsoluteRange range) {
    m_absoluteRange = Objects.requireNonNull(range, "range");
  }

  /**
   * @return the range {@link #getAbsolutePosition()} currently folds into
   */
  public synchronized AbsoluteRange getAbsoluteRange() {
    return m_absoluteRange;
  }

  /**
   * Turns position wrapping on or off for {@link #getPosition()}.
   *
   * <p>Off (the default) gives you the continuous multi-turn count. On
   * folds the reading into the range set by
   * {@link #setPositionWrappingRange(double, double)} (0-360 until you
   * change it), which is what a continuously-rotating mechanism like a
   * swerve steering module wants - there, "370 degrees" and "10 degrees"
   * are the same place and a controller should not wind up 360 degrees
   * of error trying to tell them apart.
   *
   * <p>Never turn this on for a mechanism with hard stops that can travel
   * more than one revolution (an elevator on a spool, a multi-turn
   * winch): wrapping throws away exactly the turn count that keeps it
   * from driving into its own end stop.
   */
  public synchronized void setPositionWrappingEnabled(boolean enabled) {
    m_positionWrappingEnabled = enabled;
  }

  /**
   * @return whether {@link #getPosition()} is being wrapped
   */
  public synchronized boolean isPositionWrappingEnabled() {
    return m_positionWrappingEnabled;
  }

  /**
   * Sets the range {@link #getPosition()} wraps into when wrapping is
   * enabled, in mechanism degrees. Typical choices are (0, 360) and
   * (-180, 180).
   *
   * <p>Does not enable wrapping by itself - call
   * {@link #setPositionWrappingEnabled(boolean)} too.
   *
   * @throws IllegalArgumentException if {@code maxDegrees} is not greater
   *     than {@code minDegrees}
   */
  public synchronized void setPositionWrappingRange(double minDegrees, double maxDegrees) {
    CyncoderConfig.checkPositionWrappingRange(minDegrees, maxDegrees);
    m_positionWrapMinDeg = minDegrees;
    m_positionWrapMaxDeg = maxDegrees;
  }

  /**
   * @return the lower bound set by {@link #setPositionWrappingRange(double, double)}, in degrees
   */
  public synchronized double getPositionWrappingMin() {
    return m_positionWrapMinDeg;
  }

  /**
   * @return the upper bound set by {@link #setPositionWrappingRange(double, double)}, in degrees
   */
  public synchronized double getPositionWrappingMax() {
    return m_positionWrapMaxDeg;
  }

  /**
   * How long telemetry stays valid before {@link #isConnected()} reports
   * a dropout, in seconds. Defaults to 0.1 s (ten missed 100 Hz frames).
   * Stored in whole milliseconds, rounded to the nearest.
   *
   * @throws IllegalArgumentException if the timeout is under 1 ms (frame
   *     timestamps are whole milliseconds, so anything shorter would
   *     round to zero and report every frame as stale)
   */
  public synchronized void setConnectionTimeout(double seconds) {
    CyncoderConfig.checkConnectionTimeout(seconds);
    m_connectionTimeoutMs = Math.round(seconds * 1000.0);
  }

  /**
   * @return the connection timeout set by {@link #setConnectionTimeout(double)}, in seconds
   */
  public synchronized double getConnectionTimeout() {
    return m_connectionTimeoutMs / 1000.0;
  }

  // ====================================================================
  //  WPILib unit types
  // ====================================================================
  //
  // Same values as the double getters and setters above, as Measure
  // objects: convert with .in(Rotations), .in(Radians), etc. The double
  // versions allocate nothing; prefer them in tight loops.

  /** {@link #getAbsolutePosition()} as a WPILib unit type. */
  public Angle getAbsolutePositionMeasure() {
    return Degrees.of(getAbsolutePosition());
  }

  /** {@link #getPosition()} as a WPILib unit type. */
  public Angle getPositionMeasure() {
    return Degrees.of(getPosition());
  }

  /** {@link #getVelocity()} as a WPILib unit type. */
  public AngularVelocity getVelocityMeasure() {
    return DegreesPerSecond.of(getVelocity());
  }

  /** {@link #getAbsoluteVelocity()} as a WPILib unit type. */
  public AngularVelocity getAbsoluteVelocityMeasure() {
    return DegreesPerSecond.of(getAbsoluteVelocity());
  }

  /** {@link #setPosition(double)} taking a WPILib unit type. */
  public boolean setPosition(Angle position) {
    return setPosition(position.in(Degrees));
  }

  /** {@link #setMagnetOffset(double)} taking a WPILib unit type. */
  public void setMagnetOffset(Angle offset) {
    setMagnetOffset(offset.in(Degrees));
  }

  /** {@link #setPositionOffset(double)} taking a WPILib unit type. */
  public void setPositionOffset(Angle offset) {
    setPositionOffset(offset.in(Degrees));
  }

  /** {@link #setPositionWrappingRange(double, double)} taking WPILib unit types. */
  public void setPositionWrappingRange(Angle min, Angle max) {
    setPositionWrappingRange(min.in(Degrees), max.in(Degrees));
  }

  /** {@link #setConnectionTimeout(double)} taking a WPILib unit type. */
  public void setConnectionTimeout(Time timeout) {
    setConnectionTimeout(timeout.in(Seconds));
  }

  // ====================================================================
  //  Connection and magnet health
  // ====================================================================

  /**
   * Whether a position frame has arrived within the connection timeout.
   *
   * <p>False means the board is off, unpowered, on a different CAN ID
   * than you asked for, or its wiring has come loose - the position and
   * velocity getters keep returning the last value they saw, so check
   * this before trusting them in anything that moves.
   */
  public synchronized boolean isConnected() {
    refreshPositionVelocity();
    return m_positionFresh;
  }

  /**
   * FPGA timestamp of the most recent position frame, in seconds, or -1
   * if none has ever arrived. Same timebase as
   * {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.
   */
  public synchronized double getLastUpdateTimestamp() {
    refreshPositionVelocity();
    return timestampSeconds();
  }

  /**
   * How well the magnet is seated, as judged by the firmware from the
   * sensor's own magnet status. Anything but
   * {@link CyncoderMagnetHealth#GOOD} means the position readings may be
   * drifting or jumping, even though frames are still arriving normally.
   */
  public synchronized CyncoderMagnetHealth getMagnetHealth() {
    refreshMagnetHealth();
    return m_health;
  }

  /**
   * The sensor's automatic-gain-control value, 0-255. Drifting toward
   * either extreme means the magnet is getting too far away (high AGC)
   * or too close (low AGC); mid-scale is where you want it.
   */
  public synchronized int getAGC() {
    refreshMagnetHealth();
    return m_agc;
  }

  /**
   * The sensor's raw field-strength reading, 0-4095 - a secondary
   * diagnostic alongside AGC.
   */
  public synchronized int getMagnitude() {
    refreshMagnetHealth();
    return m_magnitude;
  }

  /**
   * The CAN protocol version the board's firmware reports.
   *
   * @return {@link CyncoderCANProtocol#PROTOCOL_VERSION} or later for
   *     current firmware; 0 for firmware from before the version field
   *     existed, or if no health frame has been received yet. In
   *     simulation, {@link CyncoderCANProtocol#PROTOCOL_VERSION}.
   */
  public synchronized int getFirmwareProtocolVersion() {
    if (m_simDevice != null) {
      return CyncoderCANProtocol.PROTOCOL_VERSION;
    }
    refreshMagnetHealth();
    return m_protocolVersion;
  }

  /**
   * How many times the board has gone CAN bus-off and recovered since it
   * powered on, saturating at 255. A count that keeps rising points at
   * the bus itself: wiring, termination, or two devices sharing an ID.
   *
   * @return the count, or -1 if the firmware does not report it (protocol
   *     version 0) or no health frame has been received. 0 in simulation.
   */
  public synchronized int getBusOffCount() {
    if (m_simDevice != null) {
      return 0;
    }
    refreshMagnetHealth();
    return m_busOffCount;
  }

  /**
   * Enables or disables this encoder's WPILib {@link Alert}s ("Cyncoder N
   * disconnected", "Cyncoder N magnet ..."), shown in the "Alerts" group on
   * dashboards such as Elastic. On by default. Alerts update whenever the
   * encoder is read, so they follow any subsystem that reads it every loop.
   */
  public synchronized void setAlertsEnabled(boolean enabled) {
    m_alertsEnabled = enabled;
    updateAlerts(); // clears them now if called on the owner thread, else on its next read
  }

  /**
   * Blocks until the encoder is reporting, or {@code timeoutSeconds}
   * passes. Use it once at startup before seeding anything from
   * {@link #getAbsolutePosition()}, so a mechanism is never initialized
   * from the 0 degrees an encoder reads before its first frame. Checks
   * every 5 ms.
   *
   * @param timeoutSeconds longest time to wait
   * @return {@link #isConnected()} at the end of the wait
   */
  public boolean waitForConnection(double timeoutSeconds) {
    long startNs = System.nanoTime();
    long timeoutNs = toBoundedNanos(timeoutSeconds);
    while (true) {
      boolean connected = isConnected();
      if (connected || System.nanoTime() - startNs >= timeoutNs) {
        return connected;
      }
      try {
        Thread.sleep(5);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return isConnected();
      }
    }
  }

  /**
   * The board's unique hardware ID, folded down from the STM32's
   * factory-programmed UID, or -1 if no identity frame has arrived yet
   * (the board broadcasts one every 500 ms).
   *
   * <p>Unlike the CAN ID, this is fixed for the life of the board and
   * unique across boards, which is what lets the commands below address
   * one specific device even if two of them are temporarily sharing a
   * CAN ID.
   */
  public synchronized long getDeviceUID() {
    if (m_simDevice != null) {
      return m_deviceId; // stand-in identity; simulation has no real STM32 UID
    }
    if (readLatest(CyncoderCANProtocol.API_ID_DEVICE_UID, m_uidData) && m_uidData.length >= 4) {
      m_deviceUid = CyncoderCANProtocol.unpackDeviceUid(m_uidData.data);
    }
    return m_deviceUid;
  }

  // ====================================================================
  //  Commands to the board
  // ====================================================================

  /**
   * Tells the board that its current physical angle is zero, and to keep
   * that across power cycles.
   *
   * <p>This writes the board's Flash - it is the same operation as the
   * Tuner's "Set Zero" button, not a code-side offset. Call it from a
   * calibration routine, never from a periodic method.
   *
   * <p>The new zero takes effect on the board within a frame or two; the
   * getters here start reporting it on their own once the next telemetry
   * frame lands.
   *
   * <p>The board answers with an ACK once the zero is saved to Flash. Follow
   * the outcome with {@link #getCommandStatus()}, or block briefly with
   * {@link #waitForCommand(double)} from a calibration command.
   *
   * @return false if the board's UID isn't known yet (it has not been
   *     seen on the bus - wait up to 500 ms after enabling and retry) or
   *     if the frame could not be queued; true if the command was sent.
   *     True means "sent", not "applied" - that is what
   *     {@link #getCommandStatus()} reports.
   */
  public boolean setZero() {
    return sendTargetedCommand(
        CyncoderCANProtocol.API_ID_SET_ZERO, CyncoderCANProtocol.API_INDEX_SET_ZERO);
  }

  /**
   * Blinks the board's status LED so you can tell which physical board
   * answers to this CAN ID. Same as the Tuner's identify button.
   *
   * @return false if the board's UID isn't known yet or the frame could
   *     not be queued, true if the command was sent; see
   *     {@link #getCommandStatus()} for the board's answer
   */
  public boolean blink() {
    return sendTargetedCommand(CyncoderCANProtocol.API_ID_BLINK, CyncoderCANProtocol.API_INDEX_BLINK);
  }

  /**
   * Resets the board's stored zero point and direction-invert flag to
   * their defaults and writes that to Flash. Does not change its CAN ID,
   * and does not touch any of this class's code-side settings.
   *
   * @return false if the board's UID isn't known yet or the frame could
   *     not be queued, true if the command was sent; see
   *     {@link #getCommandStatus()} for the board's answer
   */
  public boolean factoryDefault() {
    return sendTargetedCommand(
        CyncoderCANProtocol.API_ID_FACTORY_DEFAULT, CyncoderCANProtocol.API_INDEX_FACTORY_DEFAULT);
  }

  /**
   * The outcome of the most recent {@link #setZero()}, {@link #blink()} or
   * {@link #factoryDefault()}. Non-blocking: call it from a later loop
   * until {@link CyncoderCommandStatus#isDone()}.
   *
   * <p>{@link CyncoderCommandStatus#PENDING} turns into the board's answer
   * when its ACK arrives, or into {@link CyncoderCommandStatus#TIMED_OUT}
   * after 0.5 s without one. With firmware that predates ACKs (protocol
   * version below 3) it becomes {@link CyncoderCommandStatus#UNCONFIRMED}
   * as soon as that is known. In simulation every sent command is
   * {@link CyncoderCommandStatus#APPLIED}.
   */
  public synchronized CyncoderCommandStatus getCommandStatus() {
    refreshCommandStatus();
    return m_commandStatus;
  }

  /**
   * Blocks until the most recent command has an outcome, or
   * {@code timeoutSeconds} passes. Meant for calibration commands run while
   * disabled; it can hold the calling thread for up to 0.5 s, so never
   * call it from a periodic method while enabled.
   *
   * @return the final status, or {@link CyncoderCommandStatus#PENDING} if
   *     {@code timeoutSeconds} ran out first
   */
  public CyncoderCommandStatus waitForCommand(double timeoutSeconds) {
    long startNs = System.nanoTime();
    long timeoutNs = toBoundedNanos(timeoutSeconds);
    while (true) {
      CyncoderCommandStatus status = getCommandStatus();
      if (status.isDone() || System.nanoTime() - startNs >= timeoutNs) {
        return status;
      }
      try {
        Thread.sleep(2);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return getCommandStatus();
      }
    }
  }

  // Deliberately not exposed: SET_CONFIG, which changes a board's CAN ID
  // and persisted direction-invert. Renaming a device out from under the
  // robot code that addresses it is a bench operation, and one that needs
  // a UI able to show you the ID conflict it might cause - that belongs
  // in the Tuner, not in a method a robot program could call while
  // enabled.

  private synchronized boolean sendTargetedCommand(int apiId, int apiIndex) {
    if (m_closed) {
      m_commandStatus = CyncoderCommandStatus.NOT_SENT;
      return false;
    }
    if (m_simDevice != null) {
      // No board to command in simulation. A simulated dropout refuses the
      // command, as an unreachable board would on the robot.
      boolean connected = m_simConnected.get();
      m_commandStatus = connected ? CyncoderCommandStatus.APPLIED : CyncoderCommandStatus.NOT_SENT;
      return connected;
    }
    long uid = getDeviceUID();
    if (uid < 0 || m_can == null) {
      m_commandStatus = CyncoderCommandStatus.NOT_SENT;
      return false;
    }
    int sequence = m_nextSequence;
    m_nextSequence = sequence == 255 ? 1 : sequence + 1; // 0 means "no sequence"
    // Taken before sending: a fast reply must never look older than its request.
    long sentCanMs = canNowMs();
    try {
      m_can.writePacket(CyncoderCANProtocol.packTargetUid(uid, sequence), apiId);
    } catch (RuntimeException e) {
      // A full TX queue or a bus fault. The caller decides whether to
      // retry - a calibration command failing is not worth throwing out
      // of a robot loop over.
      m_commandStatus = CyncoderCommandStatus.NOT_SENT;
      return false;
    }
    m_pendingCommandIndex = apiIndex;
    m_pendingSequence = sequence;
    m_pendingUid = uid;
    m_pendingSentCanMs = sentCanMs;
    m_commandStatus = CyncoderCommandStatus.PENDING;
    return true;
  }

  private void refreshCommandStatus() {
    if (m_commandStatus != CyncoderCommandStatus.PENDING || m_can == null || m_closed) {
      return;
    }
    // Matching on sequence and UID as well as the command means a stale ACK
    // from an earlier command, or one from another board sharing this CAN
    // ID, is never mistaken for the answer to this one. The 1 ms slack
    // covers the timebase's millisecond rounding.
    if (readLatest(CyncoderCANProtocol.API_ID_COMMAND_ACK, m_ackData)
        && m_ackData.length >= 8
        && canDeltaMs(m_ackData.timestamp, m_pendingSentCanMs) >= -1) {
      CyncoderCANProtocol.CommandAck ack = CyncoderCANProtocol.unpackCommandAck(m_ackData.data);
      if (ack.commandIndex() == m_pendingCommandIndex
          && ack.sequence() == m_pendingSequence
          && ack.uid() == m_pendingUid) {
        m_commandStatus = CyncoderCommandStatus.fromAckResult(ack.result());
        return;
      }
    }
    refreshMagnetHealth(); // learns the firmware's protocol version
    if (m_healthFrameSeen
        && m_protocolVersion < CyncoderCANProtocol.FIRST_VERSION_WITH_COMMAND_ACK) {
      m_commandStatus = CyncoderCommandStatus.UNCONFIRMED;
    } else if (canDeltaMs(canNowMs(), m_pendingSentCanMs) > COMMAND_ACK_TIMEOUT_MS) {
      m_commandStatus = CyncoderCommandStatus.TIMED_OUT;
    }
  }

  // ====================================================================
  //  Internals
  // ====================================================================

  /** Sensor-shaft angle in degrees: continuous, sign-corrected, magnet-offset applied. */
  private double sensorDegrees() {
    return (m_inverted ? -m_rawPositionDeg : m_rawPositionDeg) + m_magnetOffsetDeg;
  }

  /**
   * Single-turn sensor angle, before range wrapping. Uses the board's exact
   * tick count when a current v2 health frame provides one, otherwise the
   * multi-turn float position (whose precision falls off at very large
   * turn counts).
   */
  private double absoluteSensorDegrees() {
    double baseDeg =
        absoluteTicksMatchPositionFrame()
            ? CyncoderCANProtocol.ticksToDegrees(m_absoluteTicks)
            : m_rawPositionDeg;
    return (m_inverted ? -baseDeg : baseDeg) + m_magnetOffsetDeg;
  }

  private double absolutePositionDegrees() {
    return wrapToRange(absoluteSensorDegrees(), m_absoluteRange.min, m_absoluteRange.max);
  }

  private double positionDegrees() {
    double mechanismDeg = sensorDegrees() / m_sensorToMechanismRatio + m_positionOffsetDeg;
    return m_positionWrappingEnabled
        ? wrapToRange(mechanismDeg, m_positionWrapMinDeg, m_positionWrapMaxDeg)
        : mechanismDeg;
  }

  private double velocityDegrees() {
    return sensorVelocityDegrees() / m_sensorToMechanismRatio;
  }

  private double timestampSeconds() {
    return m_lastPositionFpgaSeconds;
  }

  /**
   * Wraps {@code value} into the half-open range [min, max).
   *
   * <p>Not {@code MathUtil.inputModulus}: that returns the upper end for an
   * input exactly on a whole-turn boundary, so 0 degrees read as 360 and
   * -180 as +180 - most often right after a Set Zero, when the board
   * reports exactly 0.
   */
  static double wrapToRange(double value, double min, double max) {
    double range = max - min;
    double wrapped = (value - min) % range;
    if (wrapped < 0.0) {
      wrapped += range;
    }
    if (wrapped >= range) {
      wrapped -= range; // a tiny negative remainder can round up to exactly `range`
    }
    return wrapped + min;
  }

  /** Sensor-shaft velocity in degrees per second, sign-corrected. */
  private double sensorVelocityDegrees() {
    return m_inverted ? -m_rawVelocityDegPerSec : m_rawVelocityDegPerSec;
  }

  /**
   * Largest gap, in ms, between a position frame and the health frame whose
   * absolute tick count may be combined with it. The board sends the two
   * back to back from the same sample, so their receive times normally
   * match to the millisecond.
   */
  private static final long COMPANION_FRAME_TOLERANCE_MS = 3;

  /** Seconds to nanoseconds, clamped to [0, 1 hour]; NaN counts as 0. */
  private static long toBoundedNanos(double seconds) {
    if (!(seconds > 0.0)) {
      return 0L;
    }
    return (long) (Math.min(seconds, 3600.0) * 1e9);
  }

  /** Current time on the CAN frame timebase, in ms. */
  private static long canNowMs() {
    return CAN.getTimestampBaseTime();
  }

  /**
   * Milliseconds from {@code earlierMs} to {@code laterMs} on the 32-bit CAN
   * timebase, correct across the counter's wrap (every ~49.7 days). Negative
   * when {@code laterMs} is actually earlier.
   */
  static long canDeltaMs(long laterMs, long earlierMs) {
    return (int) (laterMs - earlierMs);
  }

  /**
   * Reads the latest frame for an API ID. Any HAL error counts as "no
   * frame" instead of propagating: a getter must never throw into a robot
   * loop because of a CAN hiccup.
   */
  private boolean readLatest(int apiId, CANData data) {
    if (m_can == null || m_closed) {
      return false;
    }
    try {
      return m_can.readPacketLatest(apiId, data);
    } catch (RuntimeException e) {
      return false;
    }
  }

  private void refreshPositionVelocity() {
    if (m_closed) {
      m_positionFresh = false;
      return;
    }
    if (m_simDevice != null) {
      // Mirror the hardware path below, including what it does when the
      // board goes quiet: freeze on the last value rather than tracking
      // a sim that keeps moving. A subsystem being tested against a
      // simulated CAN dropout should see the same frozen reading it
      // would see on the field.
      // A sensor fault stops the board's position frames on hardware, so it
      // disconnects the simulated reading too.
      m_positionFresh =
          m_simConnected.get()
              && m_simMagnetHealth.get() != CyncoderMagnetHealth.SENSOR_FAULT.getValue();
      if (m_positionFresh) {
        m_rawPositionDeg = m_simRawPositionDeg.get();
        m_rawVelocityDegPerSec = m_simRawVelocityDegPerSec.get();
        m_lastPositionFpgaSeconds = Timer.getFPGATimestamp();
      }
      updateAlerts();
      return;
    }

    if (!readLatest(CyncoderCANProtocol.API_ID_POSITION_VELOCITY, m_positionData)
        || m_positionData.length < 8) {
      // No frame has ever arrived, or the one that did is malformed.
      // Leave the cached values alone rather than snapping to zero - a
      // mechanism does not teleport to 0 degrees because a wire fell out,
      // and isConnected() is how callers learn the reading is stale.
      m_positionFresh = false;
      updateAlerts();
      return;
    }

    long ageMs = canDeltaMs(canNowMs(), m_positionData.timestamp);
    if (m_positionData.timestamp != m_positionFrameCanMs) {
      double[] positionVelocity = CyncoderCANProtocol.unpackPositionVelocity(m_positionData.data);
      if (Double.isFinite(positionVelocity[0]) && Double.isFinite(positionVelocity[1])) {
        m_rawPositionDeg = positionVelocity[0];
        m_rawVelocityDegPerSec = positionVelocity[1];
        m_positionFrameCanMs = m_positionData.timestamp;
        m_lastPositionFpgaSeconds = Timer.getFPGATimestamp() - ageMs / 1000.0;
      }
      // A corrupt (non-finite) frame is ignored: freshness is then judged
      // by the last good frame below.
    }
    m_positionFresh =
        m_positionFrameCanMs >= 0
            && ageOf(m_positionFrameCanMs) >= -1
            && ageOf(m_positionFrameCanMs) <= m_connectionTimeoutMs;
    updateAlerts();
  }

  /** Age in ms of a frame received at {@code frameCanMs}. */
  private static long ageOf(long frameCanMs) {
    return canDeltaMs(canNowMs(), frameCanMs);
  }

  private void refreshMagnetHealth() {
    if (m_closed) {
      return;
    }
    if (m_simDevice != null) {
      // A disconnected board reports no health either - same as the
      // stale-frame branch below, so a simulated dropout looks like a
      // real one from every getter, not just isConnected().
      // AGC and magnitude freeze on their last values while disconnected,
      // exactly as they do on hardware when frames stop arriving.
      if (m_simConnected.get()) {
        m_health = CyncoderMagnetHealth.fromValue(m_simMagnetHealth.get());
        m_agc = m_simAgc.get();
        m_magnitude = m_simMagnitude.get();
      } else {
        m_health = CyncoderMagnetHealth.UNKNOWN;
      }
      updateAlerts();
      return;
    }

    if (!readLatest(CyncoderCANProtocol.API_ID_MAGNET_HEALTH, m_healthData)
        || m_healthData.length < 4) {
      return; // keep the last known health; UNKNOWN until the first frame
    }

    long ageMs = ageOf(m_healthData.timestamp);
    if (ageMs < -1 || ageMs > m_connectionTimeoutMs) {
      // Health frames ride alongside position frames at the same 100 Hz,
      // so a stale one means the board is gone - report that rather than
      // a reassuring GOOD from before it dropped off. The stale absolute
      // tick count is dropped too, so the absolute angle falls back to the
      // (equally frozen) position frame instead of mixing ages.
      m_health = CyncoderMagnetHealth.UNKNOWN;
      m_absoluteTicks = -1;
      updateAlerts();
      return;
    }

    m_healthFrameSeen = true;
    m_healthFrameCanMs = m_healthData.timestamp;
    int[] health = CyncoderCANProtocol.unpackMagnetHealth(m_healthData.data);
    m_health = CyncoderMagnetHealth.fromValue(health[0]);
    m_agc = health[1];
    m_magnitude = health[2];

    int[] extension = CyncoderCANProtocol.unpackHealthExtension(m_healthData.data, m_healthData.length);
    if (extension != null) {
      m_absoluteTicks = extension[0];
      m_protocolVersion = extension[1];
      m_busOffCount = extension[2];
    } else {
      m_absoluteTicks = -1;
      m_protocolVersion = 0;
      m_busOffCount = -1;
    }
    updateAlerts();
  }

  /**
   * Whether the health frame's exact absolute ticks belong to the same
   * sample as the current position frame. Only then may they be combined,
   * so an absolute angle and a position are never from different cycles.
   */
  private boolean absoluteTicksMatchPositionFrame() {
    if (m_absoluteTicks < 0 || m_positionFrameCanMs < 0 || m_healthFrameCanMs < 0) {
      return false;
    }
    return Math.abs(canDeltaMs(m_healthFrameCanMs, m_positionFrameCanMs))
        <= COMPANION_FRAME_TOLERANCE_MS;
  }

  /**
   * Brings the alerts up to date - but only on the thread that created
   * this encoder, because WPILib's Alert is not thread-safe. From any other
   * thread this is a no-op; the next read on the owner thread catches up.
   */
  private void updateAlerts() {
    if (m_closed || Thread.currentThread() != m_ownerThread) {
      return;
    }
    m_disconnectedAlert.set(m_alertsEnabled && !m_positionFresh);

    boolean magnetFault =
        m_health == CyncoderMagnetHealth.FAR
            || m_health == CyncoderMagnetHealth.ERROR
            || m_health == CyncoderMagnetHealth.SENSOR_FAULT;
    if (magnetFault) {
      String text =
          m_health == CyncoderMagnetHealth.SENSOR_FAULT
              ? "Cyncoder " + m_deviceId + " sensor not responding"
              : "Cyncoder " + m_deviceId + " magnet " + m_health + " (AGC " + m_agc + ")";
      m_magnetAlert.setText(text);
    }
    m_magnetAlert.set(m_alertsEnabled && magnetFault);
  }

  // ====================================================================
  //  Dashboard / lifecycle
  // ====================================================================

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Cyncoder");
    builder.addDoubleProperty("Absolute Position (deg)", this::getAbsolutePosition, null);
    builder.addDoubleProperty("Position (deg)", this::getPosition, null);
    builder.addDoubleProperty("Velocity (deg per s)", this::getVelocity, null);
    builder.addDoubleProperty("Absolute Velocity (deg per s)", this::getAbsoluteVelocity, null);
    builder.addBooleanProperty("Connected", this::isConnected, null);
    builder.addStringProperty("Magnet Health", () -> getMagnetHealth().toString(), null);
    builder.addIntegerProperty("AGC", this::getAGC, null);
    builder.addIntegerProperty("Magnitude", this::getMagnitude, null);
    builder.addIntegerProperty("CAN ID", () -> m_deviceId, null);
    builder.addIntegerProperty("Firmware Protocol", this::getFirmwareProtocolVersion, null);
    builder.addIntegerProperty("Bus-Off Count", this::getBusOffCount, null);
    builder.addStringProperty("Last Command", () -> getCommandStatus().toString(), null);
  }

  /**
   * Releases the CAN handle (or simulation device), the alerts and the
   * dashboard registration. Safe to call more than once: later calls do
   * nothing. After closing, getters return the last values read and
   * report the encoder as disconnected; commands are not sent.
   */
  @Override
  public synchronized void close() {
    if (m_closed) {
      return; // a second close could free a handle since reused by another device
    }
    m_closed = true;
    m_positionFresh = false;
    if (m_commandStatus == CyncoderCommandStatus.PENDING) {
      // The command went out, but its answer can no longer be read.
      m_commandStatus = CyncoderCommandStatus.UNCONFIRMED;
    }
    SendableRegistry.remove(this);
    m_disconnectedAlert.close();
    m_magnetAlert.close();
    if (m_can != null) {
      m_can.close();
    }
    if (m_simDevice != null) {
      m_simDevice.close();
    }
  }
}
