// Cyncoder - code-side configuration, applied all at once.

package com.cygnus.CygnusLib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import java.util.Objects;

/**
 * Every code-side setting of a {@link Cyncoder}, as one value you build once
 * and apply with {@link Cyncoder#applyConfig(CyncoderConfig)} or pass to
 * {@link Cyncoder#Cyncoder(int, CyncoderConfig)}.
 *
 * <pre>{@code
 * private static final CyncoderConfig STEER_CONFIG =
 *     new CyncoderConfig()
 *         .withAbsoluteRange(Cyncoder.AbsoluteRange.PLUS_MINUS_180)
 *         .withMagnetOffset(-37.5);
 *
 * private final Cyncoder m_steer = new Cyncoder(5, STEER_CONFIG);
 * }</pre>
 *
 * <p>Each {@code with} method checks its argument immediately and throws on
 * a bad value, so a config that exists is always valid and applying it can
 * never fail halfway. The {@code with} methods modify and return this
 * object; use {@link #copy()} to branch from a shared base.
 *
 * <p>Like the individual setters, none of this is written to the board. The
 * position offset is deliberately not part of the config: it is runtime
 * state set by homing ({@link Cyncoder#setPosition(double)}), and applying a
 * config must not undo a homing.
 */
public class CyncoderConfig {
  private boolean m_inverted;
  private double m_magnetOffsetDeg;
  private double m_sensorToMechanismRatio = 1.0;
  private Cyncoder.AbsoluteRange m_absoluteRange = Cyncoder.AbsoluteRange.ZERO_TO_360;
  private boolean m_positionWrappingEnabled;
  private double m_positionWrapMinDeg;
  private double m_positionWrapMaxDeg = 360.0;
  private double m_connectionTimeoutSeconds = 0.1;
  private boolean m_alertsEnabled = true;

  /** A config holding the defaults every new {@link Cyncoder} starts with. */
  public CyncoderConfig() {}

  // ---- Builders ----------------------------------------------------------

  /** See {@link Cyncoder#setInverted(boolean)}. */
  public CyncoderConfig withInverted(boolean inverted) {
    m_inverted = inverted;
    return this;
  }

  /** See {@link Cyncoder#setMagnetOffset(double)}. */
  public CyncoderConfig withMagnetOffset(double offsetDegrees) {
    checkFinite("magnet offset", offsetDegrees);
    m_magnetOffsetDeg = offsetDegrees;
    return this;
  }

  /** See {@link Cyncoder#setMagnetOffset(double)}. */
  public CyncoderConfig withMagnetOffset(Angle offset) {
    return withMagnetOffset(offset.in(Degrees));
  }

  /**
   * See {@link Cyncoder#setSensorToMechanismRatio(double)}.
   *
   * @throws IllegalArgumentException if the ratio is not greater than zero
   */
  public CyncoderConfig withSensorToMechanismRatio(double ratio) {
    checkSensorToMechanismRatio(ratio);
    m_sensorToMechanismRatio = ratio;
    return this;
  }

  /** See {@link Cyncoder#setAbsoluteRange(Cyncoder.AbsoluteRange)}. */
  public CyncoderConfig withAbsoluteRange(Cyncoder.AbsoluteRange range) {
    m_absoluteRange = Objects.requireNonNull(range, "range");
    return this;
  }

  /** See {@link Cyncoder#setPositionWrappingEnabled(boolean)}. */
  public CyncoderConfig withPositionWrappingEnabled(boolean enabled) {
    m_positionWrappingEnabled = enabled;
    return this;
  }

  /**
   * See {@link Cyncoder#setPositionWrappingRange(double, double)}. Does not
   * enable wrapping by itself.
   *
   * @throws IllegalArgumentException unless {@code maxDegrees > minDegrees}
   */
  public CyncoderConfig withPositionWrappingRange(double minDegrees, double maxDegrees) {
    checkPositionWrappingRange(minDegrees, maxDegrees);
    m_positionWrapMinDeg = minDegrees;
    m_positionWrapMaxDeg = maxDegrees;
    return this;
  }

  /**
   * See {@link Cyncoder#setPositionWrappingRange(double, double)}. Does not
   * enable wrapping by itself.
   *
   * @throws IllegalArgumentException unless {@code max > min}
   */
  public CyncoderConfig withPositionWrappingRange(Angle min, Angle max) {
    return withPositionWrappingRange(min.in(Degrees), max.in(Degrees));
  }

  /**
   * See {@link Cyncoder#setConnectionTimeout(double)}. Rounded to whole
   * milliseconds, as the encoder applies it, so a config read back with
   * {@link Cyncoder#getConfig()} equals the one applied.
   *
   * @throws IllegalArgumentException if under 1 ms
   */
  public CyncoderConfig withConnectionTimeout(double seconds) {
    checkConnectionTimeout(seconds);
    m_connectionTimeoutSeconds = Math.round(seconds * 1000.0) / 1000.0;
    return this;
  }

  /**
   * See {@link Cyncoder#setConnectionTimeout(double)}.
   *
   * @throws IllegalArgumentException if under 1 ms
   */
  public CyncoderConfig withConnectionTimeout(Time timeout) {
    return withConnectionTimeout(timeout.in(Seconds));
  }

  /** See {@link Cyncoder#setAlertsEnabled(boolean)}. */
  public CyncoderConfig withAlertsEnabled(boolean enabled) {
    m_alertsEnabled = enabled;
    return this;
  }

  // ---- Getters -----------------------------------------------------------

  public boolean getInverted() {
    return m_inverted;
  }

  /** @return degrees */
  public double getMagnetOffset() {
    return m_magnetOffsetDeg;
  }

  public double getSensorToMechanismRatio() {
    return m_sensorToMechanismRatio;
  }

  public Cyncoder.AbsoluteRange getAbsoluteRange() {
    return m_absoluteRange;
  }

  public boolean isPositionWrappingEnabled() {
    return m_positionWrappingEnabled;
  }

  /** @return degrees */
  public double getPositionWrappingMin() {
    return m_positionWrapMinDeg;
  }

  /** @return degrees */
  public double getPositionWrappingMax() {
    return m_positionWrapMaxDeg;
  }

  /** @return seconds */
  public double getConnectionTimeout() {
    return m_connectionTimeoutSeconds;
  }

  public boolean getAlertsEnabled() {
    return m_alertsEnabled;
  }

  // ---- Value semantics ---------------------------------------------------

  /** An independent copy, for building variants from a shared base. */
  public CyncoderConfig copy() {
    return new CyncoderConfig()
        .withInverted(m_inverted)
        .withMagnetOffset(m_magnetOffsetDeg)
        .withSensorToMechanismRatio(m_sensorToMechanismRatio)
        .withAbsoluteRange(m_absoluteRange)
        .withPositionWrappingEnabled(m_positionWrappingEnabled)
        .withPositionWrappingRange(m_positionWrapMinDeg, m_positionWrapMaxDeg)
        .withConnectionTimeout(m_connectionTimeoutSeconds)
        .withAlertsEnabled(m_alertsEnabled);
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }
    if (!(other instanceof CyncoderConfig o)) {
      return false;
    }
    return m_inverted == o.m_inverted
        && Double.compare(m_magnetOffsetDeg, o.m_magnetOffsetDeg) == 0
        && Double.compare(m_sensorToMechanismRatio, o.m_sensorToMechanismRatio) == 0
        && m_absoluteRange == o.m_absoluteRange
        && m_positionWrappingEnabled == o.m_positionWrappingEnabled
        && Double.compare(m_positionWrapMinDeg, o.m_positionWrapMinDeg) == 0
        && Double.compare(m_positionWrapMaxDeg, o.m_positionWrapMaxDeg) == 0
        && Double.compare(m_connectionTimeoutSeconds, o.m_connectionTimeoutSeconds) == 0
        && m_alertsEnabled == o.m_alertsEnabled;
  }

  @Override
  public int hashCode() {
    return Objects.hash(
        m_inverted,
        m_magnetOffsetDeg,
        m_sensorToMechanismRatio,
        m_absoluteRange,
        m_positionWrappingEnabled,
        m_positionWrapMinDeg,
        m_positionWrapMaxDeg,
        m_connectionTimeoutSeconds,
        m_alertsEnabled);
  }

  @Override
  public String toString() {
    return "CyncoderConfig{inverted="
        + m_inverted
        + ", magnetOffsetDeg="
        + m_magnetOffsetDeg
        + ", sensorToMechanismRatio="
        + m_sensorToMechanismRatio
        + ", absoluteRange="
        + m_absoluteRange
        + ", positionWrapping="
        + (m_positionWrappingEnabled
            ? "[" + m_positionWrapMinDeg + ", " + m_positionWrapMaxDeg + ")"
            : "off")
        + ", connectionTimeoutSeconds="
        + m_connectionTimeoutSeconds
        + ", alertsEnabled="
        + m_alertsEnabled
        + "}";
  }

  // ---- Validation, shared with Cyncoder's setters -------------------------

  /** Frame timestamps are whole milliseconds, so a shorter timeout would round to zero. */
  static final double MIN_CONNECTION_TIMEOUT_SECONDS = 0.001;

  static void checkFinite(String name, double value) {
    if (!Double.isFinite(value)) {
      throw new IllegalArgumentException(name + " must be a finite number, got " + value);
    }
  }

  static void checkSensorToMechanismRatio(double ratio) {
    if (!(ratio > 0.0) || !Double.isFinite(ratio)) {
      throw new IllegalArgumentException("sensorToMechanismRatio must be finite and > 0, got " + ratio);
    }
  }

  static void checkPositionWrappingRange(double minDegrees, double maxDegrees) {
    checkFinite("position wrapping min", minDegrees);
    checkFinite("position wrapping max", maxDegrees);
    if (!(maxDegrees > minDegrees)) {
      throw new IllegalArgumentException(
          "position wrapping range needs max > min, got min=" + minDegrees + " max=" + maxDegrees);
    }
  }

  static void checkConnectionTimeout(double seconds) {
    if (!(seconds >= MIN_CONNECTION_TIMEOUT_SECONDS) || !Double.isFinite(seconds)) {
      throw new IllegalArgumentException(
          "connection timeout must be finite and at least 0.001 seconds, got " + seconds);
    }
  }
}
