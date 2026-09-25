// Cygnus Tuner robot-code bridge.
//
// Runs INSIDE your robot program (not a separate SSH-deployed daemon)
// as a background thread, relaying Cyncoders' raw CAN frames
// to backend/roborio_bridge.py over the same plain TCP protocol the
// standalone roborio_daemon/main.cpp already speaks - see that file's
// header comment for the wire format, reproduced here for reference.
//
// WHY THIS EXISTS ALONGSIDE roborio_daemon/main.cpp, NOT INSTEAD OF IT:
// The standalone daemon has to call HAL_Initialize() itself, which
// claims the roboRIO's FPGA/CAN session exclusively and kills whatever
// else (your robot program, another configuration tool's session) was using
// it - every practical problem this whole SSH-deploy approach has had
// (robotCommand juggling, forced reboots, "no devices" races) traces
// back to that one call. THIS class never touches HAL_Initialize at all
// - your robot program already called it via TimedRobot's normal
// startup, so CyncoderBridge just piggybacks on that existing session the
// same way vendor diagnostic servers do when a robot program is running.
// Cygnus Tuner (backend/roborio_bridge.py) tries connecting directly to
// port 5800 first, and only falls back to SSH-deploying the standalone
// daemon if nothing answers there - so having this running costs
// nothing and needs no configuration on the Tuner side.
//
// INTEGRATION: call CyncoderBridge.start() once from Robot.robotInit()
// (or your RobotContainer's constructor), and CyncoderBridge.stop() from
// Robot.close() if you override it (optional - a robot program's
// process lifetime IS the RIO's uptime in practice, so this is mostly
// for simulation/testing hygiene, not something competition code needs
// to worry about).
//
//   @Override
//   public void robotInit() {
//     CyncoderBridge.start();
//   }
//
// SAFETY: frames from the Tuner are only put on the bus if they are
// Cyncoder commands (see isAllowedOutgoingId()), the robot is disabled and
// no FMS is attached.
// Anything else is dropped with a rate-limited Driver Station warning, so
// the open port cannot be used to drive other CAN devices.
//
// Wire protocol (shared with roborio_daemon/main.cpp):
//   Bridge -> backend, once when a session starts:
//     [1]  type = 0x03 (hello)
//     [4]  ASCII "CYGB" - identifies this in-program bridge; the daemon
//          sends "CYGD". Lets the Tuner tell the two apart.
//   Bridge -> backend (a CAN frame was received):
//     [1]  type = 0x01
//     [4]  messageID    (big-endian)
//     [1]  dataSize     (0-8)
//     [4]  timeStampMs  (big-endian) - truncated to 32 bits, backend
//                                      never reads this field anyway
//     [dataSize] payload
//   backend -> bridge (send this frame on the CAN bus):
//     [1]  type = 0x02
//     [4]  messageID    (big-endian)
//     [1]  dataSize     (0-8)
//     [dataSize] payload

package com.cygnus.CygnusLib;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * Relays Cyncoder CAN traffic between the robot's CAN bus and the Cygnus
 * Tuner over TCP port 5800, from inside the robot program. See the file
 * header for the protocol and safety rules.
 */
public final class CyncoderBridge {
  private static final int LISTEN_PORT = 5800;

  // Relay every Cyncoder frame on the bus regardless of its current CAN ID:
  // device type and manufacturer fixed, API class/index/device number free.
  // Same as roborio_daemon/main.cpp's kFilterId/Mask.
  private static final int FILTER_ID =
      (CyncoderCANProtocol.DEVICE_TYPE << 24) | (CyncoderCANProtocol.MANUFACTURER << 16);
  private static final int FILTER_MASK = (0x1F << 24) | (0xFF << 16);

  // The only frames the Tuner ever sends are commands (API class 1) to a
  // Cyncoder. Anything else is refused, so a client on the robot network
  // cannot use the bridge to put arbitrary frames - motor controller
  // setpoints included - on the robot's CAN bus.
  private static final int EXTENDED_ID_MASK = 0x1FFFFFFF;

  private static final int MAX_STREAM_MESSAGES = 32;
  private static final byte MSG_CAN_FRAME = 0x01;
  private static final byte MSG_SEND_CAN_FRAME = 0x02;
  private static final byte MSG_HELLO = 0x03;
  private static final byte[] HELLO_ID = "CYGB".getBytes(StandardCharsets.US_ASCII);

  private static final long STOP_JOIN_MS = 2000;
  private static final long REJECT_WARNING_INTERVAL_NS = 2_000_000_000L;

  // Guarded by CyncoderBridge.class.
  private static Thread s_acceptThread;
  private static ServerSocket s_server;
  private static Session s_session;

  private static volatile long s_lastRejectWarningNs;

  private CyncoderBridge() {}

  /**
   * Starts the bridge: binds TCP port 5800 and serves Tuner connections on
   * a background daemon thread. Safe to call more than once - calls while
   * it is already running do nothing.
   */
  public static synchronized void start() {
    if (s_acceptThread != null) {
      return;
    }
    ServerSocket server;
    try {
      server = new ServerSocket();
      // Rebinding right after stop(), or after a robot-code restart, must
      // not fail on a socket still in TIME_WAIT.
      server.setReuseAddress(true);
      server.bind(new InetSocketAddress(LISTEN_PORT));
    } catch (IOException e) {
      DriverStation.reportError("CyncoderBridge: could not listen on port " + LISTEN_PORT + ": " + e, false);
      return;
    }
    s_server = server;
    s_acceptThread = new Thread(() -> runAcceptLoop(server), "CyncoderBridge");
    s_acceptThread.setDaemon(true);
    s_acceptThread.start();
    DriverStation.reportWarning("CyncoderBridge listening on port " + LISTEN_PORT, false);
  }

  /**
   * Stops the bridge: closes the listening socket and any connected Tuner,
   * and waits up to about 2 seconds for the bridge thread to exit. The port
   * is free for {@link #start()} again as soon as this returns.
   */
  public static void stop() {
    Thread acceptThread;
    Session session;
    synchronized (CyncoderBridge.class) {
      closeQuietly(s_server); // unblocks accept()
      session = s_session;
      acceptThread = s_acceptThread;
      s_server = null;
      s_session = null;
      s_acceptThread = null;
    }
    if (session != null) {
      session.close();
      session.join(STOP_JOIN_MS);
    }
    if (acceptThread != null) {
      try {
        acceptThread.join(STOP_JOIN_MS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }

  private static void runAcceptLoop(ServerSocket server) {
    // Loop on this thread's own socket, not shared state: after stop() and
    // a new start(), this thread must still see its socket closed and exit.
    while (!server.isClosed()) {
      Socket client;
      try {
        client = server.accept();
      } catch (IOException e) {
        if (!server.isClosed()) {
          DriverStation.reportWarning("CyncoderBridge: accept failed: " + e, false);
        }
        continue;
      }

      Session session = new Session(client);
      Session previous;
      synchronized (CyncoderBridge.class) {
        if (s_server != server) {
          session.close(); // stopped while this connection was arriving
          return;
        }
        previous = s_session;
        s_session = session;
      }
      // One Tuner at a time. A new connection replaces the old one, so a
      // laptop that vanished without closing its socket (Wi-Fi dropped)
      // can never lock later connections out.
      if (previous != null) {
        previous.close();
      }
      DriverStation.reportWarning("CyncoderBridge: Tuner connected", false);
      session.start();
    }
  }

  /**
   * Whether the bridge may put a frame with this ID on the bus: a 29-bit
   * data frame, addressed to a Cyncoder (its device type and manufacturer),
   * in the command API class.
   */
  static boolean isAllowedOutgoingId(int messageId) {
    if ((messageId & ~EXTENDED_ID_MASK) != 0) {
      return false; // remote-frame or 11-bit flag set, not a plain extended frame
    }
    if ((messageId & FILTER_MASK) != FILTER_ID) {
      return false; // some other device
    }
    int apiClass = (messageId >>> 10) & 0x3F;
    return apiClass == CyncoderCANProtocol.API_CLASS_COMMAND;
  }

  private static void closeQuietly(java.io.Closeable closeable) {
    if (closeable == null) {
      return;
    }
    try {
      closeable.close();
    } catch (IOException e) {
      // Already closed or broken - nothing more to do.
    }
  }

  private static void warnRejected(String message) {
    long now = System.nanoTime();
    if (now - s_lastRejectWarningNs >= REJECT_WARNING_INTERVAL_NS) {
      s_lastRejectWarningNs = now;
      DriverStation.reportWarning("CyncoderBridge: " + message, false);
    }
  }

  /**
   * One connected Tuner. The pump thread owns the CAN stream session and is
   * the only writer to the socket; the reader thread applies the Tuner's
   * commands. Either one ending closes the socket, which ends the other.
   */
  private static final class Session {
    private final Socket m_socket;
    private final Thread m_pump;
    private final Thread m_reader;
    private boolean m_closed; // guarded by this

    Session(Socket socket) {
      m_socket = socket;
      m_pump = new Thread(this::pump, "CyncoderBridge-pump");
      m_reader = new Thread(this::read, "CyncoderBridge-reader");
      m_pump.setDaemon(true);
      m_reader.setDaemon(true);
    }

    void start() {
      try {
        m_socket.setTcpNoDelay(true); // small, latency-sensitive frames
        m_socket.setKeepAlive(true);
      } catch (IOException e) {
        // Options are an optimization; the session still works without them.
      }
      m_pump.start();
      m_reader.start();
    }

    void close() {
      synchronized (this) {
        if (m_closed) {
          return;
        }
        m_closed = true;
      }
      closeQuietly(m_socket); // unblocks both threads' socket I/O
      m_pump.interrupt();
      synchronized (CyncoderBridge.class) {
        if (s_session == this) {
          s_session = null;
        }
      }
    }

    void join(long timeoutMs) {
      try {
        m_pump.join(timeoutMs);
        m_reader.join(timeoutMs);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }

    /** CAN -> Tuner. Owns the stream session from open to close. */
    private void pump() {
      int handle;
      try {
        handle = CANJNI.openCANStreamSession(FILTER_ID, FILTER_MASK, MAX_STREAM_MESSAGES);
      } catch (RuntimeException e) {
        DriverStation.reportError("CyncoderBridge: openCANStreamSession failed: " + e, false);
        close();
        return;
      }

      CANStreamMessage[] messages = new CANStreamMessage[MAX_STREAM_MESSAGES];
      for (int i = 0; i < messages.length; i++) {
        messages[i] = new CANStreamMessage();
      }
      try {
        OutputStream out = new BufferedOutputStream(m_socket.getOutputStream(), 4096);
        out.write(MSG_HELLO);
        out.write(HELLO_ID);
        out.flush();

        byte[] frame = new byte[10 + 8];
        while (!isClosed() && !Thread.currentThread().isInterrupted()) {
          int messagesRead;
          try {
            messagesRead = CANJNI.readCANStreamSession(handle, messages, MAX_STREAM_MESSAGES);
          } catch (RuntimeException e) {
            // CANStreamOverflowException or a transient read error - not
            // fatal, same as the standalone daemon's retry.
            messagesRead = 0;
          }

          for (int i = 0; i < messagesRead; i++) {
            CANStreamMessage msg = messages[i];
            if (msg.length < 0 || msg.length > 8) {
              continue; // not a classic CAN frame
            }
            frame[0] = MSG_CAN_FRAME;
            putU32BE(frame, 1, msg.messageID);
            frame[5] = (byte) msg.length;
            putU32BE(frame, 6, (int) msg.timestamp);
            System.arraycopy(msg.data, 0, frame, 10, msg.length);
            out.write(frame, 0, 10 + msg.length);
          }
          if (messagesRead > 0) {
            out.flush(); // one TCP write per batch, not per frame
          } else {
            Thread.sleep(1); // don't spin when the bus is quiet
          }
        }
      } catch (IOException | InterruptedException e) {
        // Tuner gone, or closing - fall through to cleanup.
      } finally {
        close();
        CANJNI.closeCANStreamSession(handle);
        DriverStation.reportWarning("CyncoderBridge: Tuner disconnected", false);
      }
    }

    /** Tuner -> CAN. Applies one command per iteration until the socket closes. */
    private void read() {
      try {
        DataInputStream in = new DataInputStream(m_socket.getInputStream());
        while (!isClosed()) {
          handleIncomingCommand(in);
        }
      } catch (IOException e) {
        // Orderly disconnect or a read error. Not logged: it happens on
        // every ordinary Tuner disconnect.
      } finally {
        close();
      }
    }

    private synchronized boolean isClosed() {
      return m_closed;
    }
  }

  /** Reads and applies exactly one backend-&gt;bridge command frame. */
  private static void handleIncomingCommand(DataInputStream in) throws IOException {
    int type = in.readUnsignedByte();
    if (type != MSG_SEND_CAN_FRAME) {
      throw new IOException("unknown message type 0x" + Integer.toHexString(type) + " from Tuner");
    }
    int messageId = in.readInt(); // DataInputStream reads big-endian, matching our wire format
    int dataSize = in.readUnsignedByte();
    if (dataSize > 8) {
      throw new IOException("bogus dataSize " + dataSize + " from Tuner");
    }
    byte[] payload = new byte[dataSize];
    in.readFully(payload);

    if (!isAllowedOutgoingId(messageId)) {
      warnRejected("refused frame id=0x" + Integer.toHexString(messageId) + " (not a Cyncoder command)");
      return;
    }
    if (DriverStation.isEnabled() || DriverStation.isFMSAttached()) {
      // Configuration writes Flash and can rename devices out from under
      // running code - never while enabled, and never on a competition field.
      warnRejected("refused Tuner command while the robot is enabled or on the field");
      return;
    }

    try {
      CANJNI.FRCNetCommCANSessionMuxSendMessage(messageId, payload, CANJNI.CAN_SEND_PERIOD_NO_REPEAT);
    } catch (RuntimeException e) {
      DriverStation.reportWarning(
          "CyncoderBridge: sendMessage failed for id=0x" + Integer.toHexString(messageId) + ": " + e, false);
    }
  }

  private static void putU32BE(byte[] buf, int offset, int value) {
    buf[offset] = (byte) (value >>> 24);
    buf[offset + 1] = (byte) (value >>> 16);
    buf[offset + 2] = (byte) (value >>> 8);
    buf[offset + 3] = (byte) value;
  }
}
