#!/usr/bin/env python3
"""
Laptop-side human detection and AI safety lease for the ESP32 rover.

Topology:
  ESP32-CAM :81/stream  --> OpenCV capture thread --> latest frame --> YOLOv8n class 0
  Laptop    --> ws://DevKit:81/ws --> {ai, seq, client_ms}

The laptop never drives motors. The DevKit remains the motor-state authority.

Safety behavior:
  - Start the AI lease in STOP state.
  - Human detection -> STOP immediately.
  - A chosen avoidance side must remain human-clear for --side-clear-hold before steering.
  - BoT-SORT is the default tracker for the moving camera; track history is retained conservatively.
  - Predictive avoidance projects human tracks forward and rejects sides that are likely to become occupied.
  - Weak-confidence forward-person detections trigger STOP rather than autonomous avoidance.
  - Both side corridors occupied -> STOP.
  - After avoidance, use slow RECOVERY while the forward path remains clear for --recovery-clear-hold.
  - Recovery speed is ramped gradually; only then return to AI_SESSION.
  - CLEAR must remain continuous for --clear-hold before normal GO when no recovery is active.
  - Stale/lost camera frames -> STOP.
  - AI packets are refreshed every --keepalive seconds (default 125 ms).
  - If the laptop process/link dies, the DevKit lease expires and fails closed.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import websocket
from ultralytics import YOLO

LOGGER = logging.getLogger("rover-ai")
CURRENT_AI_HEALTH = 0
CURRENT_FRAME_AGE_MS = None
CURRENT_INFERENCE_MS = None
CURRENT_WS_RTT_MS = None
CURRENT_VISION_FPS = None
CURRENT_OPTICAL_FLOW_MS = None
CURRENT_DECISION_AGE_MS = None


@dataclass(frozen=True)
class FrameSample:
    frame: object
    captured_at: float
    count: int


@dataclass(frozen=True)
class PacketContext:
    trigger: str
    frame_age_ms: Optional[float]
    inference_ms: Optional[float]
    clear_hold_ms: Optional[float]
    ramp_ms: Optional[float] = None
    left_blocked: Optional[bool] = None
    right_blocked: Optional[bool] = None
    side_clear_ms: Optional[float] = None
    blocked_ms: Optional[float] = None
    chosen_side: Optional[str] = None
    resume_condition: Optional[str] = None
    release_qualified: Optional[bool] = None
    release_side: Optional[str] = None
    ai_health_score: Optional[int] = None
    ai_frame_age_ms: Optional[float] = None
    ai_inference_ms: Optional[float] = None
    ai_ws_rtt_ms: Optional[float] = None
    ai_vision_fps: Optional[float] = None
    ai_optical_flow_mag: Optional[float] = None
    ai_decision_age_ms: Optional[float] = None


@dataclass(frozen=True)
class PendingPacket:
    sent_at: float
    action: str
    immediate: bool
    context: PacketContext


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ESP32-CAM YOLOv8n person detector with AI safety lease")
    parser.add_argument("--devkit-ip", default="192.168.4.1", help="DevKit AP IP")
    parser.add_argument("--cam-ip", default="192.168.4.2", help="ESP32-CAM IP")
    parser.add_argument("--ws-port", type=int, default=81, help="DevKit WebSocket port")
    parser.add_argument("--stream-port", type=int, default=81, help="Camera MJPEG port")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLOv8n weights/path")
    parser.add_argument("--confidence", type=float, default=0.45, help="Strong person confidence required for autonomous avoidance")
    parser.add_argument("--health-stop", type=int, default=35, help="AI health score at/below which autonomous motion stops")
    parser.add_argument("--health-caution", type=int, default=75, help="AI health score below which autonomous speed is reduced")
    parser.add_argument("--min-vision-fps", type=float, default=8.0, help="Expected minimum camera/vision FPS")
    parser.add_argument("--stop-confidence", type=float, default=0.25, help="Lower person confidence used to retain safety recall; weak forward detections stop rather than steer")
    parser.add_argument("--keepalive", type=float, default=0.125, help="AI packet interval in seconds")
    parser.add_argument("--lease-timeout", type=float, default=0.45, help="DevKit lease timeout expectation in seconds")
    parser.add_argument("--clear-hold", type=float, default=0.8, help="Continuous clear time required before GO")
    parser.add_argument("--frame-timeout", type=float, default=0.45, help="Maximum acceptable camera-frame age in seconds")
    parser.add_argument("--front-y", type=float, default=0.55, help="Normalized lower-image threshold for a person to count as in-front")
    parser.add_argument("--avoid-max-speed", type=float, default=0.30, help="Maximum normalized forward speed requested during human avoidance")
    parser.add_argument("--avoid-min-speed", type=float, default=0.14, help="Minimum normalized forward speed requested during human avoidance")
    parser.add_argument("--avoid-max-turn", type=float, default=0.50, help="Maximum normalized turn command during human avoidance")
    parser.add_argument("--close-stop-height", type=float, default=0.70, help="Person bbox height fraction above which hard STOP is used")
    parser.add_argument("--side-corridor", type=float, default=0.26, help="Half-width of the central side-corridor fraction used to classify blocked sides")
    parser.add_argument("--side-clear-hold", type=float, default=0.30, help="Continuous time a chosen side must remain human-clear before steering")
    parser.add_argument("--blocked-hold", type=float, default=0.30, help="Continuous time both side corridors must remain human-occupied before BLOCKED_BY_HUMANS")
    parser.add_argument("--recovery-clear-hold", type=float, default=0.90, help="Continuous time the forward path must remain clear during recovery before AI_SESSION")
    parser.add_argument("--recovery-start-speed", type=float, default=0.08, help="Normalized speed at recovery entry")
    parser.add_argument("--recovery-max-speed", type=float, default=0.18, help="Maximum normalized slow recovery speed")
    parser.add_argument("--recovery-ramp", type=float, default=1.50, help="Seconds used to ramp from slow recovery speed to normal AI cruise speed after recovery clear hold")
    parser.add_argument("--ai-cruise-speed", type=float, default=0.30, help="Normal autonomous AI forward speed after recovery")
    parser.add_argument("--io-timeout", type=float, default=0.05, help="WebSocket receive timeout used by network thread")
    parser.add_argument("--reconnect", type=float, default=0.25, help="WebSocket reconnect interval")
    parser.add_argument("--imgsz", type=int, default=416, help="YOLO inference image size")
    parser.add_argument("--tracker", default="botsort.yaml", help="Ultralytics tracker config; BoT-SORT adds moving-camera compensation")
    parser.add_argument("--track-memory", type=float, default=0.35, help="Seconds to retain a recently tracked forward human as a conservative safety memory")
    parser.add_argument("--freeze-timeout", type=float, default=0.35, help="Seconds of near-identical camera frames while moving before STOP")
    parser.add_argument("--command-stale-timeout", type=float, default=0.25, help="Seconds after the last vision update before the network thread forces STOP instead of replaying drive/avoid/recover")
    parser.add_argument("--prediction-horizon", type=float, default=0.35, help="Seconds used for track-motion prediction in side-risk evaluation")
    parser.add_argument("--prediction-margin", type=float, default=0.06, help="Normalized margin added around predicted person boxes")
    parser.add_argument("--approach-area-rate", type=float, default=0.70, help="Normalized bbox area growth/sec triggering rapid-approach STOP")
    parser.add_argument("--optical-flow-sample-every", type=int, default=3, help="Compute optical-flow health every N processed frames")
    parser.add_argument("--optical-flow-jump", type=float, default=18.0, help="Median optical-flow magnitude jump treated as a vision instability event")
    parser.add_argument("--optical-flow-freeze", type=float, default=0.35, help="Median optical-flow magnitude below this while driving contributes a camera-health penalty")
    parser.add_argument("--avoid-timeout", type=float, default=3.0, help="Maximum continuous avoidance duration before fail-closed STOP")
    parser.add_argument("--avoid-switch-window", type=float, default=2.0, help="Time window for avoidance-side switch detection")
    parser.add_argument("--avoid-switch-max", type=int, default=2, help="Maximum side switches in the window before STOP")
    parser.add_argument("--show", action="store_true", help="Show annotated detection window")
    return parser.parse_args()


class LatestFrameCapture:
    """Continuously drain the MJPEG stream and keep exactly one newest frame."""

    def __init__(self, url: str, reconnect_delay: float = 0.25) -> None:
        self.url = url
        self.reconnect_delay = reconnect_delay
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[FrameSample] = None
        self._thread = threading.Thread(target=self._run, name="camera-capture", daemon=True)
        self._cap: Optional[cv2.VideoCapture] = None
        self._count = 0
        self._last_error_log = 0.0
        self._fps = 0.0
        self._fps_window_started = time.monotonic()
        self._fps_window_count = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        cap = self._cap
        if cap is not None:
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
        self._thread.join(timeout=1.0)

    def latest(self) -> Optional[FrameSample]:
        with self._lock:
            return self._latest

    @property
    def fps(self) -> float:
        return self._fps

    def _open(self) -> Optional[cv2.VideoCapture]:
        now = time.monotonic()
        if now - self._last_error_log >= 1.0:
            LOGGER.info("Opening camera MJPEG stream: %s", self.url)
            self._last_error_log = now
        cap = cv2.VideoCapture(self.url)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # pragma: no cover
            pass
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
            return None
        LOGGER.info("Camera stream connected")
        return cap

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._cap is None or not self._cap.isOpened():
                self._cap = self._open()
                if self._cap is None:
                    self._stop.wait(self.reconnect_delay)
                    continue

            ok, frame = self._cap.read()
            if not ok or frame is None:
                LOGGER.warning("Camera frame read failed; reconnecting")
                try:
                    self._cap.release()
                except Exception:  # noqa: BLE001
                    pass
                self._cap = None
                self._stop.wait(self.reconnect_delay)
                continue

            self._count += 1
            captured_now = time.monotonic()
            self._fps_window_count += 1
            window_dt = captured_now - self._fps_window_started
            if window_dt >= 1.0:
                self._fps = self._fps_window_count / window_dt
                self._fps_window_started = captured_now
                self._fps_window_count = 0
            sample = FrameSample(frame=frame, captured_at=captured_now, count=self._count)
            with self._lock:
                self._latest = sample

        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:  # noqa: BLE001
                pass
            self._cap = None


class DevKitAIChannel:
    """Asynchronous AI sender/receiver with sequence numbers and ACK timing.

    AI actions:
      stop    -> hard safety stop
      go      -> finish a clear/recovery transition; DevKit handles gating
      avoid   -> bounded slow forward + turn after side-clear confirmation
      recover -> bounded recovery forward motion
      drive   -> normal autonomous AI forward/turn control
    """

    def __init__(
        self,
        url: str,
        keepalive: float,
        io_timeout: float,
        reconnect_delay: float,
        command_stale_timeout: float = 0.25,
    ) -> None:
        self.url = url
        self.keepalive = keepalive
        self.io_timeout = io_timeout
        self.reconnect_delay = reconnect_delay
        self._command_stale_timeout = max(0.10, min(0.40, command_stale_timeout))

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._thread = threading.Thread(
            target=self._network_loop,
            name="ai-network",
            daemon=True,
        )

        self._ws: Optional[websocket.WebSocket] = None
        self._desired_action = "stop"
        self._desired_reason = "startup"
        self._desired_speed = 0.0
        self._desired_turn = 0.0
        self._desired_context = PacketContext(
            "startup", None, None, None
        )
        self._immediate = True

        # Sequence space is per Python process. The DevKit resets its
        # acceptance baseline whenever a new AI session is established.
        self._seq = 0

        self._pending: dict[int, PendingPacket] = {}
        self._connected = False
        self._last_connect_error = 0.0
        self._last_keepalive_log = 0.0
        self._manual_override = False
        self._session_requires_stop = True
        self._last_vision_update = time.monotonic()
        self._last_rtt_ms = 0.0

    @property
    def desired_action(self) -> str:
        with self._lock:
            return self._desired_action

    @property
    def desired_stop(self) -> bool:
        return self.desired_action == "stop"

    @property
    def manual_override(self) -> bool:
        with self._lock:
            return self._manual_override

    @property
    def last_rtt_ms(self) -> float:
        with self._lock:
            return self._last_rtt_ms

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected and self._ws is not None

    def start(self) -> None:
        self._thread.start()
        self._wake_event.set()

    def set_command(
        self,
        action: str,
        reason: str,
        context: PacketContext,
        speed: float = 0.0,
        turn: float = 0.0,
    ) -> None:
        if action not in {"stop", "go", "avoid", "recover", "drive"}:
            raise ValueError(f"unsupported AI action: {action}")

        with self._lock:
            changed = (
                action != self._desired_action
                or abs(speed - self._desired_speed) > 1e-3
                or abs(turn - self._desired_turn) > 1e-3
            )

            self._desired_action = action
            self._desired_reason = reason
            self._desired_speed = float(speed)
            self._desired_turn = float(turn)
            self._desired_context = context
            self._last_vision_update = time.monotonic()

            if changed:
                self._immediate = True

        if changed:
            LOGGER.info(
                "AI command request: %s reason=%s speed=%.2f turn=%.2f",
                action.upper(),
                reason,
                speed,
                turn,
            )
            self._wake_event.set()

    def set_state(
        self,
        stop: bool,
        reason: str,
        context: PacketContext,
    ) -> None:
        """Compatibility helper for callers using the old boolean interface."""
        self.set_command(
            "stop" if stop else "go",
            reason,
            context,
        )

    def close(self) -> None:
        self._stop_event.set()
        self._wake_event.set()
        self._close_socket()
        self._thread.join(timeout=1.0)

    def _close_socket(self) -> None:
        with self._lock:
            ws = self._ws
            self._ws = None
            self._connected = False

        if ws is not None:
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass

    def _connect(self) -> bool:
        self._close_socket()

        try:
            ws = websocket.create_connection(
                self.url,
                timeout=max(0.25, self.io_timeout * 4.0),
                enable_multithread=True,
            )
            ws.settimeout(self.io_timeout)

            with self._lock:
                self._ws = ws
                self._connected = True
                # Every new TCP/WebSocket session must re-establish the
                # DevKit AI lease with a STOP handshake before the previous
                # desired action can be sent again.
                self._session_requires_stop = True
                self._immediate = True

            LOGGER.info(
                "DevKit WebSocket connected: %s",
                self.url,
            )
            return True

        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()

            if now - self._last_connect_error >= 1.0:
                LOGGER.error(
                    "DevKit WebSocket connection failed: %s",
                    exc,
                )
                self._last_connect_error = now

            return False

    def _send_current(self, immediate: bool) -> None:
        with self._lock:
            ws = self._ws
            if self._session_requires_stop:
                action = "stop"
                reason = "reconnect_handshake"
                speed = 0.0
                turn = 0.0
                context = PacketContext(
                    "reconnect_handshake", None, None, None
                )
            else:
                action = self._desired_action
                reason = self._desired_reason
                speed = self._desired_speed
                turn = self._desired_turn
                context = self._desired_context
                vision_age = time.monotonic() - self._last_vision_update

            # Independent network-thread failsafe: if inference/capture stops
            # refreshing the command, never keep replaying an old drive command.
            if action in {"drive", "avoid", "recover"} and vision_age > self._command_stale_timeout:
                action = "stop"
                reason = "vision_command_stale"
                speed = 0.0
                turn = 0.0
                context = PacketContext(
                    "vision_command_stale",
                    None,
                    None,
                    None,
                )
                LOGGER.error(
                    "VISION COMMAND STALE age=%.0fms -> forcing STOP heartbeat",
                    vision_age * 1000.0,
                )

            self._seq = (self._seq + 1) & 0xFFFFFFFF
            seq = self._seq

        if ws is None:
            with self._lock:
                self._immediate = True
            return

        client_ms = (
            int(time.monotonic() * 1000.0)
            & 0xFFFFFFFF
        )

        packet = {
            "ai": action,
            "seq": seq,
            "client_ms": client_ms,
            "reason": reason,
        }

        # The first AI handshake explicitly selects AUTONOMOUS mode. This is
        # a mode selection only; the DevKit still remains stopped until the
        # valid AI lease/drive state machine permits motion.
        if action == "stop" and reason == "reconnect_handshake":
            packet["mode"] = "auto"

        # Vision/safety metadata is sent with every AI packet so the DevKit
        # can expose live side occupancy and hold timers to the dashboard.
        if context.left_blocked is not None:
            packet["left_blocked"] = bool(context.left_blocked)
        if context.right_blocked is not None:
            packet["right_blocked"] = bool(context.right_blocked)
        if context.side_clear_ms is not None:
            packet["side_clear_ms"] = int(max(0.0, context.side_clear_ms))
        elif context.clear_hold_ms is not None and action in {"avoid", "go", "stop"}:
            packet["side_clear_ms"] = int(max(0.0, context.clear_hold_ms))
        if context.blocked_ms is not None:
            packet["blocked_ms"] = int(max(0.0, context.blocked_ms))
        if context.chosen_side is not None:
            packet["chosen_side"] = context.chosen_side
        if context.resume_condition is not None:
            packet["resume_condition"] = context.resume_condition
        if context.release_qualified is not None:
            packet["release_qualified"] = bool(context.release_qualified)
        if context.release_side is not None:
            packet["release_side"] = context.release_side

        # Camera pan servo: in autonomous avoidance the camera follows the
        # selected clear side; during normal driving it returns to center.
        if action == "avoid" and context.chosen_side in {"left", "right"}:
            packet["servo"] = context.chosen_side
        elif action in {"go", "drive", "recover", "stop"}:
            packet["servo"] = "center"

        packet["ai_health_score"] = int(max(0, min(100, CURRENT_AI_HEALTH)))
        packet["ai_frame_age_ms"] = int(max(0.0, context.ai_frame_age_ms if context.ai_frame_age_ms is not None else (CURRENT_FRAME_AGE_MS or 0.0)))
        packet["ai_inference_ms"] = int(max(0.0, context.ai_inference_ms if context.ai_inference_ms is not None else (CURRENT_INFERENCE_MS or 0.0)))
        packet["ai_ws_rtt_ms"] = int(max(0.0, CURRENT_WS_RTT_MS or 0.0))
        packet["ai_vision_fps"] = int(max(0.0, CURRENT_VISION_FPS or 0.0))
        packet["ai_optical_flow_mag"] = float(max(0.0, context.ai_optical_flow_mag if context.ai_optical_flow_mag is not None else (CURRENT_OPTICAL_FLOW_MS or 0.0)))
        packet["ai_decision_age_ms"] = int(max(0.0, context.ai_decision_age_ms if context.ai_decision_age_ms is not None else (CURRENT_DECISION_AGE_MS or 0.0)))

        if action == "avoid":
            packet["speed"] = float(speed)
            packet["turn"] = float(turn)
            packet["side_clear_ms"] = int(
                max(0.0, context.side_clear_ms if context.side_clear_ms is not None
                    else (context.clear_hold_ms or 0.0))
            )
        elif action == "go":
            packet["clear_ms"] = int(
                max(0.0, context.clear_hold_ms or 0.0)
            )
        elif action == "recover":
            packet["speed"] = float(speed)
        elif action == "drive":
            packet["speed"] = float(speed)
            packet["turn"] = float(turn)
            packet["clear_ms"] = int(
                max(0.0, context.clear_hold_ms or 0.0)
            )
            packet["ramp_ms"] = int(
                max(0.0, context.ramp_ms or 0.0)
            )

        message = json.dumps(
            packet,
            separators=(",", ":"),
        )

        send_start = time.monotonic()

        try:
            ws.send(message)

            send_call_ms = (
                time.monotonic() - send_start
            ) * 1000.0

            with self._lock:
                self._pending[seq] = PendingPacket(
                    sent_at=send_start,
                    action=action,
                    immediate=immediate,
                    context=context,
                )

                if len(self._pending) > 64:
                    oldest = min(self._pending)
                    self._pending.pop(oldest, None)

                self._immediate = False

            LOGGER.info(
                "AI -> DevKit %s seq=%d send_call=%.1fms "
                "trigger=%s frame_age=%s inference=%s "
                "speed=%.2f turn=%.2f",
                action.upper(),
                seq,
                send_call_ms,
                context.trigger,
                (
                    f"{context.frame_age_ms:.1f}ms"
                    if context.frame_age_ms is not None
                    else "--"
                ),
                (
                    f"{context.inference_ms:.1f}ms"
                    if context.inference_ms is not None
                    else "--"
                ),
                speed,
                turn,
            )

        except Exception as exc:  # noqa: BLE001
            LOGGER.error(
                "AI command send failed seq=%d: %s",
                seq,
                exc,
            )

            with self._lock:
                self._immediate = True

            self._close_socket()

    def _handle_message(self, message: str) -> None:
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            LOGGER.warning(
                "DevKit sent non-JSON WebSocket data: %r",
                message,
            )
            return

        msg_type = data.get("type")

        if msg_type == "ai_ack":
            try:
                seq = int(data.get("seq"))
            except (TypeError, ValueError):
                return

            with self._lock:
                pending = self._pending.pop(seq, None)

            if pending is None:
                return

            rtt_ms = (
                time.monotonic() - pending.sent_at
            ) * 1000.0

            with self._lock:
                self._last_rtt_ms = rtt_ms

            accepted = bool(
                data.get("accepted", False)
            )

            reason = data.get(
                "reason",
                "unknown",
            )

            if not accepted:
                LOGGER.error(
                    "AI ACK REJECTED seq=%d action=%s "
                    "reason=%s RTT=%.1fms",
                    seq,
                    pending.action,
                    reason,
                    rtt_ms,
                )
                return

            if pending.action == "stop":
                with self._lock:
                    self._session_requires_stop = False

            if pending.immediate:
                LOGGER.info(
                    "AI ACK seq=%d action=%s RTT=%.1fms "
                    "state=%s stop_reason=%s",
                    seq,
                    pending.action.upper(),
                    rtt_ms,
                    data.get("state", "?"),
                    data.get("stop_reason", "?"),
                )
            elif (
                time.monotonic() -
                self._last_keepalive_log >= 2.0
            ):
                LOGGER.info(
                    "AI heartbeat healthy seq=%d action=%s "
                    "RTT=%.1fms lease_age=%sms state=%s",
                    seq,
                    pending.action.upper(),
                    rtt_ms,
                    data.get("ai_age_ms", "?"),
                    data.get("state", "?"),
                )
                self._last_keepalive_log = (
                    time.monotonic()
                )

        elif msg_type == "ai_state":
            state = data.get("state")
            if state == "manual":
                with self._lock:
                    self._manual_override = True
                LOGGER.warning(
                    "MANUAL OVERRIDE reported by DevKit; AI command transmission paused"
                )

            LOGGER.info(
                "DevKit AI state: stopped=%s "
                "state=%s reason=%s lease=%s age=%sms seq=%s",
                data.get("stopped"),
                data.get("state"),
                data.get("reason"),
                data.get("ai_lease"),
                data.get("ai_age_ms"),
                data.get("seq"),
            )

    def _network_loop(self) -> None:
        next_keepalive = 0.0
        next_connect = 0.0

        while not self._stop_event.is_set():
            now = time.monotonic()

            with self._lock:
                manual_override = self._manual_override
                connected = (
                    self._connected
                    and self._ws is not None
                )
                immediate_pending = self._immediate

            if manual_override and not connected:
                self._stop_event.wait(0.05)
                continue

            if not connected:
                if now >= next_connect:
                    if self._connect():
                        next_keepalive = 0.0
                        continue

                    next_connect = (
                        now + self.reconnect_delay
                    )

                self._stop_event.wait(0.02)
                continue

            if not manual_override and (immediate_pending or now >= next_keepalive):
                self._send_current(
                    immediate=immediate_pending
                )
                next_keepalive = (
                    time.monotonic()
                    + self.keepalive
                )

            with self._lock:
                ws = self._ws

            if ws is None:
                continue

            try:
                incoming = ws.recv()
                if incoming:
                    self._handle_message(incoming)

            except websocket.WebSocketTimeoutException:
                pass

            except (
                websocket.WebSocketConnectionClosedException,
                websocket.WebSocketException,
                OSError,
            ) as exc:
                LOGGER.error(
                    "DevKit WebSocket disconnected: %s",
                    exc,
                )
                self._close_socket()
                next_connect = (
                    time.monotonic()
                    + self.reconnect_delay
                )


@dataclass(frozen=True)
class TrackState:
    updated_at: float
    cx: float
    cy: float
    bw: float
    bh: float
    vx: float
    vy: float
    area_growth: float


@dataclass(frozen=True)
class PersonDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    track_id: Optional[int] = None
    predicted: bool = False

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)


def choose_avoidance(
    result,
    frame_shape,
    args: argparse.Namespace,
    locked_side: Optional[str] = None,
    track_history: Optional[dict[int, TrackState]] = None,
    now: Optional[float] = None,
) -> tuple[str, float, float, dict]:
    """Predictive, conservative human avoidance.

    Prediction only removes unsafe sides or causes STOP. It never directly
    authorizes motion; side-clear hold and DevKit validation remain mandatory.
    """
    h, w = frame_shape[:2]
    boxes = result.boxes
    people: list[PersonDetection] = []
    if boxes is not None and len(boxes):
        xyxy = boxes.xyxy.cpu().tolist()
        confs = boxes.conf.cpu().tolist()
        box_ids = getattr(boxes, "id", None)
        ids = box_ids.cpu().tolist() if box_ids is not None else [None] * len(xyxy)
        for coords, conf, tid in zip(xyxy, confs, ids):
            x1, y1, x2, y2 = map(float, coords)
            try:
                track_id = int(tid) if tid is not None else None
            except (TypeError, ValueError):
                track_id = None
            people.append(PersonDetection(x1, y1, x2, y2, float(conf), track_id))

    def diag(**kwargs):
        base = {
            "target": None,
            "left_blocked": False,
            "right_blocked": False,
            "chosen_side": None,
            "side_clear_now": False,
            "forward_people": 0,
            "predicted_people": 0,
            "predicted_forward": False,
            "rapid_approach": False,
            "track_ids": [],
        }
        base.update(kwargs)
        return base

    if not people:
        return "go", 0.0, 0.0, diag(reason="no_person")

    def norm(p: PersonDetection):
        return p.x1 / w, p.y1 / h, p.x2 / w, p.y2 / h

    predicted_people: list[PersonDetection] = []
    history_now = now if now is not None else time.monotonic()
    if track_history is not None:
        stale_ids = [tid for tid, st in track_history.items() if history_now - st.updated_at > 2.0]
        for tid in stale_ids:
            track_history.pop(tid, None)
    track_ids: list[int] = []
    rapid_target = None

    if track_history is not None:
        for p in people:
            if p.track_id is None:
                continue
            tid = p.track_id
            track_ids.append(tid)
            cx = p.cx / w
            cy = ((p.y1 + p.y2) * 0.5) / h
            bw = max(0.001, p.width / w)
            bh = max(0.001, p.height / h)
            area = bw * bh
            prev = track_history.get(tid)
            vx = vy = area_growth = 0.0
            if prev is not None:
                dt = history_now - prev.updated_at
                if 0.02 <= dt <= 0.7:
                    vx = (cx - prev.cx) / dt
                    vy = (cy - prev.cy) / dt
                    area_growth = (area - prev.bw * prev.bh) / dt
            track_history[tid] = TrackState(history_now, cx, cy, bw, bh, vx, vy, area_growth)

            _, _, _, current_bottom = norm(p)
            if area_growth >= getattr(args, "approach_area_rate", 0.70) and (
                current_bottom >= args.front_y or 0.25 <= cx <= 0.75
            ):
                rapid_target = p

            horizon = max(0.0, min(1.0, getattr(args, "prediction_horizon", 0.35)))
            if prev is not None and 0.02 <= history_now - prev.updated_at <= 0.7:
                pcx = max(0.0, min(1.0, cx + vx * horizon))
                pcy = max(0.0, min(1.0, cy + vy * horizon))
                growth = max(0.85, min(1.35, 1.0 + area_growth * horizon * 0.25))
                pbw = min(1.0, bw * growth)
                pbh = min(1.0, bh * growth)
                margin = max(0.0, min(0.15, getattr(args, "prediction_margin", 0.06)))
                px1 = max(0.0, pcx - pbw * 0.5 - margin) * w
                px2 = min(1.0, pcx + pbw * 0.5 + margin) * w
                py1 = max(0.0, pcy - pbh * 0.5 - margin) * h
                py2 = min(1.0, pcy + pbh * 0.5 + margin) * h
                predicted_people.append(PersonDetection(px1, py1, px2, py2, p.confidence, tid, True))

    def in_forward(p: PersonDetection) -> bool:
        _, _, _, y2n = norm(p)
        return y2n >= args.front_y or (p.height / h) >= 0.28 or 0.25 <= p.cx / w <= 0.75

    # Side corridors are evaluated independently of the forward-path test so
    # a person just outside the forward ROI can still block an avoidance side.
    side_half_gap = max(0.08, min(0.20, args.side_corridor * 0.5))
    left_lo, left_hi = 0.04, 0.50 - side_half_gap
    right_lo, right_hi = 0.50 + side_half_gap, 0.96

    def blocks_side(p: PersonDetection, lo: float, hi: float) -> bool:
        x1n, _, x2n, y2n = norm(p)
        overlap = max(0.0, min(x2n, hi) - max(x1n, lo))
        width = max(0.001, x2n - x1n)
        bottom_relevant = y2n >= args.front_y
        large_enough = p.height / h >= 0.22
        return overlap / width >= 0.20 and (bottom_relevant or large_enough)

    left_current = any(blocks_side(p, left_lo, left_hi) for p in people)
    right_current = any(blocks_side(p, right_lo, right_hi) for p in people)
    left_pred = any(blocks_side(p, left_lo, left_hi) for p in predicted_people)
    right_pred = any(blocks_side(p, right_lo, right_hi) for p in predicted_people)
    left_risk = left_current or left_pred
    right_risk = right_current or right_pred

    in_front = [p for p in people if in_forward(p)]
    predicted_forward_people = [p for p in predicted_people if in_forward(p)]
    if rapid_target is not None:
        return "stop", 0.0, 0.0, diag(target=rapid_target, left_blocked=left_risk, right_blocked=right_risk, forward_people=max(1, len(in_front)), predicted_people=len(predicted_people), predicted_forward=bool(predicted_forward_people), rapid_approach=True, reason="rapid_human_approach", track_ids=track_ids)
    if not in_front:
        if predicted_forward_people:
            pp = predicted_forward_people[0]
            return "stop", 0.0, 0.0, diag(target=pp, left_blocked=left_risk, right_blocked=right_risk, predicted_people=len(predicted_people), predicted_forward=True, reason="predicted_forward_human", track_ids=track_ids)
        if left_risk and right_risk:
            return "blocked_candidate", 0.0, 0.0, diag(left_blocked=True, right_blocked=True, predicted_people=len(predicted_people), reason="both_side_corridors_occupied", track_ids=track_ids)
        return "go", 0.0, 0.0, diag(left_blocked=left_risk, right_blocked=right_risk, reason="person_not_in_path", predicted_people=len(predicted_people), track_ids=track_ids)

    target = max(in_front, key=lambda p: (p.height / h, p.y2 / h, p.width / w))
    target_cx = target.cx / w
    target_height = target.height / h
    target_bottom = target.y2 / h
    confidence = float(getattr(args, "confidence", 0.45))
    if target.confidence < confidence:
        return "stop", 0.0, 0.0, diag(target=target, left_blocked=True, right_blocked=True, reason="low_confidence_forward_human", forward_people=len(in_front), track_ids=track_ids)
    if target_height >= args.close_stop_height or target_bottom >= 0.96:
        return "stop", 0.0, 0.0, diag(target=target, left_blocked=True, right_blocked=True, reason="target_too_close", forward_people=len(in_front), track_ids=track_ids)

    if left_risk and right_risk:
        return "blocked_candidate", 0.0, 0.0, diag(target=target, left_blocked=True, right_blocked=True, reason="both_sides_blocked_or_predicted", forward_people=len(in_front), predicted_people=len(predicted_people), predicted_forward=bool(predicted_forward_people), track_ids=track_ids)

    if locked_side in {"left", "right"}:
        if (locked_side == "left" and left_risk) or (locked_side == "right" and right_risk):
            return "stop", 0.0, 0.0, diag(target=target, left_blocked=left_risk, right_blocked=right_risk, reason="locked_side_predicted_blocked", forward_people=len(in_front), predicted_people=len(predicted_people), track_ids=track_ids)
        desired_side = locked_side
        already_confirmed = True
    else:
        already_confirmed = False
        if 0.38 <= target_cx <= 0.62:
            candidates = ["right", "left"]
        elif target_cx < 0.5:
            candidates = ["right", "left"]
        else:
            candidates = ["left", "right"]
        desired_side = next((side for side in candidates if not ((side == "left" and left_risk) or (side == "right" and right_risk))), None)
        if desired_side is None:
            return "blocked_candidate", 0.0, 0.0, diag(target=target, left_blocked=True, right_blocked=True, reason="no_safe_predicted_side", forward_people=len(in_front), predicted_people=len(predicted_people), track_ids=track_ids)

    closeness = max(target_height / max(0.01, args.close_stop_height), target_bottom)
    closeness = max(0.0, min(1.0, closeness))
    speed = args.avoid_max_speed - closeness * (args.avoid_max_speed - args.avoid_min_speed)
    offset = abs(target_cx - 0.5) * 2.0
    turn_mag = min(args.avoid_max_turn, 0.30 + 0.20 * max(0.0, min(1.0, 1.0 - offset)))
    turn = turn_mag if desired_side == "right" else -turn_mag
    return "avoid" if already_confirmed else "avoid_candidate", max(0.0, min(args.avoid_max_speed, speed)), turn, diag(target=target, left_blocked=left_risk, right_blocked=right_risk, chosen_side=desired_side, side_clear_now=True, reason=(f"avoid_{desired_side}_locked" if already_confirmed else f"candidate_{desired_side}"), target_cx=target_cx, target_height=target_height, target_bottom=target_bottom, forward_people=len(in_front), predicted_people=len(predicted_people), predicted_forward=bool(predicted_forward_people), track_ids=track_ids, predicted_left_blocked=left_pred, predicted_right_blocked=right_pred)

def open_model(args: argparse.Namespace) -> YOLO:
    LOGGER.info("Loading YOLO model: %s", args.model)
    model = YOLO(args.model)
    LOGGER.info("YOLO model loaded")
    return model



def safety_context(
    trigger: str,
    frame_age_ms: Optional[float],
    inference_ms: Optional[float],
    diag: dict,
    *,
    clear_hold_ms: Optional[float] = None,
    ramp_ms: Optional[float] = None,
    blocked_ms: Optional[float] = None,
    resume_condition: Optional[str] = None,
    release_qualified: Optional[bool] = None,
    release_side: Optional[str] = None,
    ai_health_score: Optional[int] = None,
    ai_frame_age_ms: Optional[float] = None,
    ai_inference_ms: Optional[float] = None,
    ai_ws_rtt_ms: Optional[float] = None,
    ai_vision_fps: Optional[float] = None,
    ai_optical_flow_mag: Optional[float] = None,
    ai_decision_age_ms: Optional[float] = None,
) -> PacketContext:
    """Build a packet context with the live side-zone safety diagnostics."""
    return PacketContext(
        trigger,
        frame_age_ms,
        inference_ms,
        clear_hold_ms,
        ramp_ms,
        bool(diag.get("left_blocked")) if "left_blocked" in diag else None,
        bool(diag.get("right_blocked")) if "right_blocked" in diag else None,
        clear_hold_ms,
        blocked_ms,
        diag.get("chosen_side"),
        resume_condition,
        release_qualified,
        release_side,
        ai_health_score if ai_health_score is not None else CURRENT_AI_HEALTH,
        ai_frame_age_ms if ai_frame_age_ms is not None else CURRENT_FRAME_AGE_MS,
        ai_inference_ms if ai_inference_ms is not None else CURRENT_INFERENCE_MS,
        ai_ws_rtt_ms if ai_ws_rtt_ms is not None else CURRENT_WS_RTT_MS,
        ai_vision_fps if ai_vision_fps is not None else CURRENT_VISION_FPS,
        ai_optical_flow_mag if ai_optical_flow_mag is not None else CURRENT_OPTICAL_FLOW_MS,
        ai_decision_age_ms if ai_decision_age_ms is not None else CURRENT_DECISION_AGE_MS,
    )


def resume_condition_text(
    args: argparse.Namespace,
    *,
    blocked_ms: Optional[float] = None,
    side: Optional[str] = None,
) -> str:
    hold = int(round(args.side_clear_hold * 1000.0))
    if side in {"left", "right"}:
        return (
            f"{side.upper()} side must remain human-clear for {hold} ms; "
            "then slow avoidance/recovery resumes"
        )
    return (
        f"Either LEFT or RIGHT side must remain human-clear for {hold} ms; "
        "then slow avoidance/recovery resumes"
    )


def release_qualification(
    diag: dict,
    side_clear_ms: float,
    args: argparse.Namespace,
) -> tuple[bool, Optional[str]]:
    """Return whether one candidate side has completed the full clear hold."""
    side = diag.get("chosen_side")
    if side not in {"left", "right"}:
        return False, None

    if side == "left" and diag.get("left_blocked"):
        return False, None
    if side == "right" and diag.get("right_blocked"):
        return False, None

    qualified = side_clear_ms >= args.side_clear_hold * 1000.0
    return qualified, (side if qualified else None)


def compute_ai_health(
    frame_age_ms: float,
    inference_ms: float,
    vision_fps: float,
    ws_rtt_ms: float,
    *,
    min_vision_fps: float = 8.0,
    optical_flow_mag: Optional[float] = None,
    optical_flow_jump: bool = False,
    optical_flow_freeze: float = 0.35,
    freeze_fault: bool = False,
    tracker_fault: bool = False,
) -> int:
    if freeze_fault or tracker_fault:
        return 0
    score = 100.0
    score -= min(45.0, max(0.0, (frame_age_ms - 40.0) / 6.0))
    score -= min(25.0, max(0.0, (inference_ms - 40.0) / 5.5))
    score -= min(15.0, max(0.0, (ws_rtt_ms - 10.0) / 5.0))
    if vision_fps < min_vision_fps:
        score -= min(15.0, (min_vision_fps - max(0.0, vision_fps)) * 3.0)
    if optical_flow_jump:
        score -= 20.0
    if optical_flow_mag is not None and optical_flow_mag < optical_flow_freeze and vision_fps >= min_vision_fps:
        score -= 10.0
    return int(max(0, min(100, round(score))))

def health_speed_scale(score: int, stop_threshold: int) -> float:
    if score <= stop_threshold: return 0.0
    if score >= 90: return 1.0
    t = (score - stop_threshold) / max(1, 90 - stop_threshold)
    return 0.45 + 0.55 * max(0.0, min(1.0, t))

def main() -> int:
    global CURRENT_AI_HEALTH, CURRENT_FRAME_AGE_MS, CURRENT_INFERENCE_MS, CURRENT_WS_RTT_MS, CURRENT_VISION_FPS, CURRENT_OPTICAL_FLOW_MS, CURRENT_DECISION_AGE_MS
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")

    LOGGER.info("Laptop must be connected to DevKit AP: ESP32-Rover-AP")
    if args.command_stale_timeout >= args.lease_timeout:
        LOGGER.warning("command-stale-timeout %.0fms >= lease-timeout %.0fms; this should be lower for an independent laptop-side stop layer", args.command_stale_timeout*1000.0, args.lease_timeout*1000.0)
    args.command_stale_timeout = min(args.command_stale_timeout, max(0.10, args.lease_timeout - 0.05))
    LOGGER.info(
        "Safety timing: keepalive=%.0fms, lease=%.0fms, cmd_stale=%.0fms, clear_hold=%.0fms, side_hold=%.0fms, recovery_hold=%.0fms, frame_timeout=%.0fms",
        args.keepalive * 1000.0,
        args.lease_timeout * 1000.0,
        args.command_stale_timeout * 1000.0,
        args.clear_hold * 1000.0,
        args.side_clear_hold * 1000.0,
        args.recovery_clear_hold * 1000.0,
        args.frame_timeout * 1000.0,
    )

    stream_url = f"http://{args.cam_ip}:{args.stream_port}/stream"
    ws_url = f"ws://{args.devkit_ip}:{args.ws_port}/ws"

    model = open_model(args)
    capture = LatestFrameCapture(stream_url, reconnect_delay=args.reconnect)
    ai_channel = DevKitAIChannel(
        ws_url,
        keepalive=args.keepalive,
        io_timeout=args.io_timeout,
        reconnect_delay=args.reconnect,
        command_stale_timeout=args.command_stale_timeout,
    )

    # Start in STOP until a fresh camera frame has been observed and remained clear.
    ai_channel.start()
    ai_channel.set_command(
        "stop",
        "startup",
        PacketContext("startup", None, None, None),
    )
    capture.start()

    clear_since: Optional[float] = None
    side_clear_since: Optional[float] = None
    side_candidate: Optional[str] = None
    recovery_mode = False
    recovery_started: Optional[float] = None
    recovery_clear_since: Optional[float] = None
    recovery_normal_ramp_started: Optional[float] = None
    both_blocked_since: Optional[float] = None
    blocked_by_humans_active = False
    avoidance_active = False
    last_frame_count = 0
    last_log = time.monotonic()
    stale_stop_logged = False
    inference_count = 0
    last_action = "stop"
    last_avoid_turn_cmd = 0.0
    avoidance_started_at: Optional[float] = None
    avoidance_switch_times: list[float] = []
    locked_avoid_side: Optional[str] = None
    recent_forward_seen_until = 0.0
    recent_forward_signature = None
    frozen_since: Optional[float] = None
    track_history: dict[int, tuple[float, float, float, float, float]] = {}

    try:
        while True:
            now = time.monotonic()
            sample = capture.latest()

            if sample is None:
                clear_since = None

                if not stale_stop_logged:
                    LOGGER.warning(
                        "No camera frame yet -> STOP"
                    )
                    stale_stop_logged = True

                ai_channel.set_command(
                    "stop",
                    "camera_stale",
                    PacketContext(
                        "camera_stale",
                        None,
                        None,
                        None,
                        release_qualified=False,
                        release_side=None,
                    ),
                )

                time.sleep(0.01)
                continue

            frame_age_ms = (
                now - sample.captured_at
            ) * 1000.0

            if frame_age_ms > args.frame_timeout * 1000.0:
                clear_since = None

                if not stale_stop_logged:
                    LOGGER.warning(
                        "Camera frame stale: age=%.1fms -> STOP",
                        frame_age_ms,
                    )
                    stale_stop_logged = True

                ai_channel.set_command(
                    "stop",
                    "camera_stale",
                    PacketContext(
                        "camera_stale",
                        frame_age_ms,
                        None,
                        None,
                        release_qualified=False,
                        release_side=None,
                    ),
                )

                time.sleep(0.01)
                continue

            stale_stop_logged = False

            infer_start = time.monotonic()

            try:
                results = model.track(
                    source=sample.frame,
                    persist=True,
                    tracker=args.tracker,
                    classes=[0],
                    conf=min(args.stop_confidence, args.confidence),
                    imgsz=args.imgsz,
                    verbose=False,
                    stream_buffer=False,
                )
            except Exception as exc:  # noqa: BLE001
                LOGGER.exception("YOLO inference/tracking failure -> STOP: %s", exc)
                clear_since = None
                side_clear_since = None
                recovery_mode = False
                avoidance_active = False
                locked_avoid_side = None
                ai_channel.set_command(
                    "stop",
                    "vision_inference_fault",
                    PacketContext("vision_inference_fault", None, None, None, release_qualified=False, release_side=None),
                )
                time.sleep(0.02)
                continue

            inference_ms = (
                time.monotonic() - infer_start
            ) * 1000.0

            inference_count += 1

            post_age_ms = (
                time.monotonic() -
                sample.captured_at
            ) * 1000.0
            ws_rtt_ms = ai_channel.last_rtt_ms

            # Lightweight optical-flow consistency check. It is advisory
            # unless it indicates a true capture/inference fault; it can
            # reduce autonomous speed but never authorizes motion.
            optical_flow_jump = False
            if inference_count % max(1, args.optical_flow_sample_every) == 0:
                gray = cv2.cvtColor(
                    cv2.resize(sample.frame, (96, 54), interpolation=cv2.INTER_AREA),
                    cv2.COLOR_BGR2GRAY,
                )
                if previous_flow_gray is not None:
                    try:
                        flow = cv2.calcOpticalFlowFarneback(
                            previous_flow_gray, gray, None, 0.5, 2, 15, 2, 5, 1.2, 0
                        )
                        magnitude = cv2.magnitude(flow[..., 0], flow[..., 1])
                        optical_flow_mag = float(np.median(magnitude))
                        if optical_flow_mag > args.optical_flow_jump:
                            optical_flow_jump = True
                    except cv2.error:
                        optical_flow_mag = 0.0
                        optical_flow_jump = True
                previous_flow_gray = gray

            ai_health = compute_ai_health(
                post_age_ms, inference_ms, capture.fps, ws_rtt_ms,
                min_vision_fps=args.min_vision_fps,
                optical_flow_mag=optical_flow_mag,
                optical_flow_jump=optical_flow_jump,
                optical_flow_freeze=args.optical_flow_freeze,
            )
            vision_speed_scale = health_speed_scale(ai_health, args.health_stop)
            CURRENT_AI_HEALTH = ai_health
            CURRENT_FRAME_AGE_MS = post_age_ms
            CURRENT_INFERENCE_MS = inference_ms
            CURRENT_WS_RTT_MS = ws_rtt_ms
            CURRENT_VISION_FPS = capture.fps
            CURRENT_OPTICAL_FLOW_MS = optical_flow_mag

            # Camera-content watchdog: if the rover is supposed to be moving
            # but the video content is effectively frozen, fail closed.
            tiny = cv2.cvtColor(
                cv2.resize(sample.frame, (32, 18), interpolation=cv2.INTER_AREA),
                cv2.COLOR_BGR2GRAY,
            ).astype(np.float32)
            if recent_forward_signature is not None:
                delta = float(np.mean(np.abs(tiny - recent_forward_signature)))
                moving_command = ai_channel.desired_action in {"drive", "avoid", "recover"}
                if moving_command and delta < 0.35:
                    if frozen_since is None:
                        frozen_since = now
                else:
                    frozen_since = None
            recent_forward_signature = tiny

            if frozen_since is not None and now - frozen_since > args.freeze_timeout:
                LOGGER.error("Camera content appears frozen while AI motion was requested -> STOP")
                ai_channel.set_command(
                    "stop",
                    "camera_content_frozen",
                    PacketContext("camera_content_frozen", post_age_ms, inference_ms, None, release_qualified=False, release_side=None),
                )
                clear_since = None
                side_clear_since = None
                avoidance_active = False
                locked_avoid_side = None
                time.sleep(0.02)
                continue

            if not results:
                LOGGER.error("YOLO returned no result objects -> STOP")
                clear_since = None
                side_clear_since = None
                recovery_mode = False
                avoidance_active = False
                locked_avoid_side = None
                ai_channel.set_command(
                    "stop",
                    "vision_empty_result",
                    PacketContext(
                        "vision_empty_result",
                        post_age_ms,
                        inference_ms,
                        None,
                        release_qualified=False,
                        release_side=None,
                    ),
                )
                time.sleep(0.02)
                continue

            result = results[0]

            action, avoid_speed, avoid_turn, diag = choose_avoidance(
                result,
                sample.frame.shape,
                args,
                locked_side=locked_avoid_side,
                track_history=track_history,
                now=now,
            )
            CURRENT_DECISION_AGE_MS = (time.monotonic() - sample.captured_at) * 1000.0
            avoid_speed *= vision_speed_scale

            if avoidance_started_at is not None and now - avoidance_started_at > args.avoid_timeout:
                LOGGER.error("AVOIDANCE TIMEOUT %.2fs -> STOP", args.avoid_timeout)
                avoidance_active = False
                locked_avoid_side = None
                side_candidate = None
                side_clear_since = None
                avoidance_started_at = None
                avoidance_switch_times.clear()
                ai_channel.set_command(
                    "stop",
                    "avoidance_timeout",
                    safety_context(
                        "avoidance_timeout",
                        post_age_ms, inference_ms, diag,
                        resume_condition="Avoidance timed out; require a fresh AI safety cycle",
                        release_qualified=False, release_side=None,
                    ),
                )
                time.sleep(0.01)
                continue

            detected = diag["target"] is not None
            now = time.monotonic()

            if detected and diag.get("forward_people", 0) > 0:
                recent_forward_seen_until = now + args.track_memory

            # Conservative temporal safety memory: one missed detector frame
            # does not immediately turn a recently-seen forward human into GO.
            if (not detected) and now < recent_forward_seen_until:
                diag = dict(diag)
                diag["reason"] = "recent_forward_human_memory"
                diag["target"] = None
                action = "stop"

            # A blocked-by-humans latch may not jump directly to GO merely
            # because a frame contains no forward person. First select a
            # side and hold it human-clear for the configured interval.
            if blocked_by_humans_active and action == "go":
                chosen_side = locked_avoid_side or side_candidate or "right"
                diag = dict(diag)
                diag["chosen_side"] = chosen_side
                diag["side_clear_now"] = True
                diag["reason"] = "blocked_recovery_side_clear"
                action = "avoid_candidate"
                avoid_speed = args.avoid_min_speed
                turn_mag = min(args.avoid_max_turn, max(args.avoid_min_speed, 0.12))
                avoid_turn = turn_mag if chosen_side == "right" else -turn_mag

            # -----------------------------------------------------------
            # Hard stop conditions from the current vision frame.
            # -----------------------------------------------------------
            if post_age_ms > args.frame_timeout * 1000.0:
                clear_since = None
                side_clear_since = None
                side_candidate = None
                recovery_mode = False
                recovery_started = None
                recovery_clear_since = None
                recovery_normal_ramp_started = None
                both_blocked_since = None
                locked_avoid_side = None
                avoidance_active = False

                ai_channel.set_command(
                    "stop",
                    "camera_stale_after_inference",
                    PacketContext(
                        "camera_stale_after_inference",
                        post_age_ms,
                        inference_ms,
                        None,
                        release_qualified=False,
                        release_side=None,
                    ),
                )

            elif action == "blocked_candidate":
                # Hard-stop immediately while confirming that BOTH side corridors
                # remain human-occupied. A distinct BLOCKED_BY_HUMANS state is
                # reported only after the configured hold period.
                clear_since = None
                side_clear_since = None
                side_candidate = None
                recovery_mode = False
                recovery_started = None
                recovery_clear_since = None
                recovery_normal_ramp_started = None
                locked_avoid_side = None
                avoidance_active = False

                if both_blocked_since is None:
                    both_blocked_since = now
                    LOGGER.warning(
                        "Both side corridors blocked; hard-stopping and starting %.0fms confirmation",
                        args.blocked_hold * 1000.0,
                    )

                blocked_ms = (now - both_blocked_since) * 1000.0

                condition = resume_condition_text(args, blocked_ms=blocked_ms)
                if blocked_ms >= args.blocked_hold * 1000.0:
                    if not blocked_by_humans_active:
                        LOGGER.error(
                            "HUMANS BLOCKED BOTH SIDES for %.0fms -> HARD STOP LATCH",
                            blocked_ms,
                        )
                    blocked_by_humans_active = True
                    ai_channel.set_command(
                        "stop",
                        "blocked_by_humans",
                        safety_context(
                            "blocked_by_humans",
                            post_age_ms,
                            inference_ms,
                            diag,
                            clear_hold_ms=0.0,
                            blocked_ms=blocked_ms,
                            resume_condition=condition,
                            release_qualified=False,
                            release_side=None,
                        ),
                    )
                else:
                    ai_channel.set_command(
                        "stop",
                        "both_sides_blocked_pending",
                        safety_context(
                            "both_sides_blocked_pending",
                            post_age_ms,
                            inference_ms,
                            diag,
                            clear_hold_ms=0.0,
                            blocked_ms=blocked_ms,
                            resume_condition=condition,
                            release_qualified=False,
                            release_side=None,
                        ),
                    )

            elif action == "stop":
                clear_since = None
                side_clear_since = None
                both_blocked_since = None
                blocked_by_humans_active = False
                side_candidate = None
                recovery_mode = False
                recovery_started = None
                recovery_clear_since = None
                recovery_normal_ramp_started = None
                both_blocked_since = None
                locked_avoid_side = None
                avoidance_active = False

                ai_channel.set_command(
                    "stop",
                    str(diag["reason"]),
                    safety_context(
                        str(diag["reason"]),
                        post_age_ms,
                        inference_ms,
                        diag,
                        resume_condition=(
                            "Human detected: rover stopped; either side must be re-confirmed clear before avoidance"
                            if detected else "Camera/vision stop condition"
                        ),
                        release_qualified=False,
                        release_side=None,
                    ),
                )

            elif action == "avoid":
                both_blocked_since = None
                # If this follows BLOCKED_BY_HUMANS, keep the local blocked
                # latch until this confirmed AVOID is actually sent.
                # Side was already confirmed on a previous frame. Continue the
                # committed avoidance until the target clears or the side becomes
                # blocked. A new side is never selected without a fresh hold.
                recovery_mode = False
                recovery_started = None
                recovery_clear_since = None
                clear_since = None
                avoidance_active = True
                locked_avoid_side = diag.get("chosen_side") or locked_avoid_side
                last_avoid_turn_cmd = avoid_turn

                blocked_by_humans_active = False
                ai_channel.set_command(
                    "avoid",
                    str(diag["reason"]),
                    safety_context(
                        str(diag["reason"]),
                        post_age_ms,
                        inference_ms,
                        diag,
                        clear_hold_ms=args.side_clear_hold * 1000.0,
                        resume_condition=resume_condition_text(
                            args, side=locked_avoid_side
                        ),
                        release_qualified=True,
                        release_side=locked_avoid_side,
                    ),
                    speed=avoid_speed,
                    turn=avoid_turn,
                )

            elif action == "avoid_candidate":
                both_blocked_since = None
                # Preserve BLOCKED_BY_HUMANS locally until the side-clear hold
                # succeeds and a confirmed AVOID command is issued.
                # A new obstacle cancels recovery. We first hold the candidate
                # side clear for a short interval before allowing steering.
                recovery_mode = False
                recovery_started = None
                recovery_clear_since = None
                clear_since = None

                chosen_side = diag.get("chosen_side")
                if chosen_side != side_candidate:
                    side_candidate = chosen_side
                    side_clear_since = now
                    LOGGER.info(
                        "Avoid side candidate=%s; starting %.0fms clear hold",
                        chosen_side,
                        args.side_clear_hold * 1000.0,
                    )
                elif not diag.get("side_clear_now", False):
                    side_clear_since = None

                side_clear_ms = (
                    (now - side_clear_since) * 1000.0
                    if side_clear_since is not None
                    else 0.0
                )

                release_qualified, release_side = release_qualification(
                    diag, side_clear_ms, args
                )

                if (
                    chosen_side in {"left", "right"}
                    and diag.get("side_clear_now", False)
                    and release_qualified
                ):
                    locked_avoid_side = chosen_side
                    avoidance_active = True
                    if avoidance_started_at is None:
                        avoidance_started_at = now
                    blocked_by_humans_active = False
                    side_candidate = None
                    side_clear_since = None
                    last_avoid_turn_cmd = avoid_turn

                    ai_channel.set_command(
                        "avoid",
                        f"avoid_{chosen_side}_confirmed",
                        safety_context(
                            f"avoid_{chosen_side}_confirmed",
                            post_age_ms,
                            inference_ms,
                            diag,
                            clear_hold_ms=side_clear_ms,
                            resume_condition=resume_condition_text(
                                args, side=chosen_side
                            ),
                            release_qualified=True,
                            release_side=release_side,
                        ),
                        speed=avoid_speed,
                        turn=avoid_turn,
                    )
                else:
                    # Never steer until the side-clear hold is complete.
                    # Keep the DevKit in BLOCKED_BY_HUMANS if that was the
                    # previously latched state; the STOP packet does not
                    # release it.
                    ai_channel.set_command(
                        "stop",
                        "side_clear_pending",
                        safety_context(
                            "side_clear_pending",
                            post_age_ms,
                            inference_ms,
                            diag,
                            clear_hold_ms=side_clear_ms,
                            resume_condition=resume_condition_text(
                                args, side=chosen_side
                            ),
                            release_qualified=False,
                            release_side=None,
                        ),
                    )

            else:
                # ---------------------------------------------------------
                # Forward path is clear in the current frame.
                # If we previously executed an avoidance, enter slow
                # recovery instead of going straight back to AI_SESSION.
                # ---------------------------------------------------------
                if avoidance_active:
                    # Actual avoidance has completed; begin slow recovery.
                    recovery_mode = True
                    avoidance_started_at = None
                    avoidance_switch_times.clear()
                    locked_avoid_side = None
                    side_candidate = None
                    side_clear_since = None
                    both_blocked_since = None
                    blocked_by_humans_active = False

                    if recovery_started is None:
                        recovery_started = now
                        recovery_clear_since = now
                        recovery_normal_ramp_started = None
                        LOGGER.info(
                            "Forward path clear after avoidance -> slow recovery"
                        )
                    elif recovery_clear_since is None:
                        recovery_clear_since = now

                    recovery_clear_ms = (now - recovery_clear_since) * 1000.0

                    if recovery_clear_ms < args.recovery_clear_hold * 1000.0:
                        recovery_normal_ramp_started = None
                        recovery_speed = max(
                            0.0,
                            min(args.recovery_start_speed, args.recovery_max_speed),
                        ) * vision_speed_scale
                        ai_channel.set_command(
                            "recover",
                            "recovery_clear_pending",
                            safety_context(
                                "recovery_clear_pending",
                                post_age_ms,
                                inference_ms,
                                diag,
                                clear_hold_ms=recovery_clear_ms,
                                resume_condition=(
                                    f"Forward path must remain clear for {args.recovery_clear_hold*1000:.0f} ms; then ramp to normal AI speed over {args.recovery_ramp:.2f} s"
                                ),
                                release_qualified=False,
                                release_side=None,
                            ),
                            speed=recovery_speed,
                        )
                    else:
                        # The forward path has been continuously clear long enough.
                        # ONLY NOW start the configurable ramp toward normal AI speed.
                        if recovery_normal_ramp_started is None:
                            recovery_normal_ramp_started = now
                            LOGGER.info(
                                "Recovery clear hold satisfied -> starting %.0fms ramp %.2f -> %.2f",
                                args.recovery_ramp * 1000.0,
                                args.recovery_max_speed,
                                args.ai_cruise_speed,
                            )

                        ramp_ms = (now - recovery_normal_ramp_started) * 1000.0
                        ramp_fraction = max(
                            0.0,
                            min(1.0, ramp_ms / max(args.recovery_ramp * 1000.0, 1.0)),
                        )
                        recovery_speed = (
                            args.recovery_max_speed
                            + ramp_fraction
                            * (args.ai_cruise_speed - args.recovery_max_speed)
                        ) * vision_speed_scale

                        if ramp_fraction >= 1.0:
                            ai_channel.set_command(
                                "drive",
                                "recovery_to_ai_complete",
                                safety_context(
                                    "recovery_to_ai_complete",
                                    post_age_ms,
                                    inference_ms,
                                    diag,
                                    clear_hold_ms=recovery_clear_ms,
                                    ramp_ms=ramp_ms,
                                    resume_condition="Recovery complete; normal AI driving active",
                                    release_qualified=False,
                                    release_side=None,
                                ),
                                speed=args.ai_cruise_speed * vision_speed_scale,
                                turn=0.0,
                            )
                            LOGGER.info(
                                "Recovery normal-speed ramp complete: %.2fs clear+ramp -> AI DRIVE %.2f",
                                ramp_ms / 1000.0,
                                args.ai_cruise_speed,
                            )
                            recovery_mode = False
                            recovery_started = None
                            recovery_clear_since = None
                            recovery_normal_ramp_started = None
                            avoidance_active = False
                            avoidance_started_at = None
                            avoidance_switch_times.clear()
                            locked_avoid_side = None
                            side_candidate = None
                            side_clear_since = None
                            last_avoid_turn_cmd = 0.0
                        else:
                            ai_channel.set_command(
                                "recover",
                                "recovery_to_ai_ramp",
                                safety_context(
                                    "recovery_to_ai_ramp",
                                    post_age_ms,
                                    inference_ms,
                                    diag,
                                    clear_hold_ms=recovery_clear_ms,
                                    ramp_ms=ramp_ms,
                                    resume_condition=(
                                        f"Normal AI speed ramps from {args.recovery_max_speed:.2f} to {args.ai_cruise_speed:.2f} over {args.recovery_ramp:.2f} s"
                                    ),
                                    release_qualified=False,
                                    release_side=None,
                                ),
                                speed=recovery_speed,
                                turn=0.0,
                            )

                else:
                    # Normal AI supervision with the existing clear hysteresis.
                    side_candidate = None
                    side_clear_since = None
                    if clear_since is None:
                        clear_since = now
                        LOGGER.info(
                            "Path clear candidate: starting %.0fms clear hold",
                            args.clear_hold * 1000.0,
                        )

                    clear_hold_ms = (
                        now - clear_since
                    ) * 1000.0

                    if clear_hold_ms >= args.clear_hold * 1000.0:
                        ai_channel.set_command(
                            "drive",
                            "clear_hold_satisfied",
                            safety_context(
                                "clear_hold_satisfied",
                                post_age_ms,
                                inference_ms,
                                diag,
                                clear_hold_ms=clear_hold_ms,
                                resume_condition="Forward path clear; normal AI driving permitted",
                                release_qualified=False,
                                release_side=None,
                            ),
                            speed=args.ai_cruise_speed * vision_speed_scale,
                            turn=0.0,
                        )
                    else:
                        # Remain stopped during the normal clear-hold window.
                        ai_channel.set_command(
                            "stop",
                            "clear_pending",
                            safety_context(
                                "clear_pending",
                                post_age_ms,
                                inference_ms,
                                diag,
                                clear_hold_ms=clear_hold_ms,
                                resume_condition=(
                                    f"Forward path must remain clear for {args.clear_hold*1000:.0f} ms before normal AI driving"
                                ),
                                release_qualified=False,
                                release_side=None,
                            ),
                        )

            if sample.count != last_frame_count:
                last_frame_count = sample.count

            desired_action = ai_channel.desired_action

            target = diag.get("target")
            target_desc = "none"

            if target is not None:
                target_desc = (
                    f"cx={target.cx / sample.frame.shape[1]:.2f} "
                    f"h={target.height / sample.frame.shape[0]:.2f}"
                )

            if (
                desired_action != last_action
                or time.monotonic() - last_log >= 2.0
            ):
                LOGGER.info(
                    "VISION action=%s frame=%d age=%.1fms "
                    "inference=%.1fms effective_age=%.1fms "
                    "AI=%s target=%s side=%s left_blocked=%s "
                    "right_blocked=%s reason=%s speed=%.2f turn=%.2f",
                    "PERSON" if detected else "CLEAR",
                    sample.count,
                    frame_age_ms,
                    inference_ms,
                    post_age_ms,
                    desired_action.upper(),
                    target_desc,
                    diag.get("chosen_side"),
                    diag.get("left_blocked"),
                    diag.get("right_blocked"),
                    diag.get("reason"),
                    diag.get("track_ids", []),
                    diag.get("predicted_people", 0),
                    avoid_speed,
                    avoid_turn,
                )

                last_log = time.monotonic()
                last_action = desired_action

            if inference_count % max(1, int(2.0 / max(0.05, args.keepalive))) == 0:
                LOGGER.info(
                    "HEALTH score=%d scale=%.2f frame_age=%.0fms infer=%.0fms fps=%.1f rtt=%.1fms flow=%.2f decision_age=%.0fms action=%s reason=%s",
                    ai_health, vision_speed_scale, post_age_ms, inference_ms, capture.fps,
                    ws_rtt_ms, optical_flow_mag, CURRENT_DECISION_AGE_MS or 0.0, action.upper(), diag.get("reason", ""),
                )

            if args.show:
                annotated = result.plot()

                if detected:
                    label = (
                        f"{desired_action.upper()} "
                        f"| {diag.get('reason')}"
                    )
                elif desired_action == "recover":
                    label = (
                        f"RECOVERY/HOLD "
                        f"| age {post_age_ms:.0f}ms"
                    )
                else:
                    label = (
                        f"CLEAR/HOLD "
                        f"| age {post_age_ms:.0f}ms"
                    )

                cv2.putText(
                    annotated,
                    label,
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255) if desired_action == "stop"
                    else (
                        (0, 165, 255)
                        if desired_action == "avoid"
                        else (0, 180, 255) if desired_action == "recover"
                        else (0, 255, 0)
                    ),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    annotated,
                    (
                        f"age={post_age_ms:.0f}ms "
                        f"infer={inference_ms:.0f}ms "
                        f"speed={avoid_speed:.2f} "
                        f"turn={avoid_turn:+.2f}"
                    ),
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                cv2.imshow(
                    "ESP32 Rover - YOLOv8n "
                    "Human Avoidance",
                    annotated,
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (27, ord("q")):
                    LOGGER.info(
                        "Exit requested"
                    )
                    break


    except KeyboardInterrupt:
        LOGGER.info("Interrupted by user")
    finally:
        LOGGER.warning("Shutting down AI node; DevKit will fail closed on AI disconnect")
        ai_channel.set_command(
            "stop",
            "shutdown",
            PacketContext("shutdown", None, None, None),
        )
        time.sleep(min(0.05, args.keepalive))
        ai_channel.close()
        capture.stop()
        if args.show:
            cv2.destroyAllWindows()

    LOGGER.info("Inference frames processed: %d", inference_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
