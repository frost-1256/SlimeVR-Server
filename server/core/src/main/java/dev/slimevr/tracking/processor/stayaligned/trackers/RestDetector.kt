package dev.slimevr.tracking.processor.stayaligned.trackers

import dev.slimevr.math.Angle
import io.github.axisangles.ktmath.Quaternion
import kotlin.time.Duration
import kotlin.time.TimeSource

/**
 * Detects whether a tracker is at rest.
 *
 * A tracker is at rest when it stays within a certain rotational range for a given
 * amount of time. If it rotates past that range, it is no longer at rest.
 *
 * Entering and leaving rest use different ranges: drift is slow, so a locked
 * tracker often drifts a few degrees over several minutes and must not be
 * unlocked by it. Real motion crosses the wider exit range almost immediately.
 *
 * TODO: In practice this is good enough for Stay Aligned, but we could also consider
 * 		acceleration if we want to make this a general purpose rest detector.
 */
class RestDetector(
	private val enterRestRotation: Angle,
	private val exitRestRotation: Angle,
	private val enterRestTime: Duration,
	private val enterMovingTime: Duration,
	private val timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
) {
	enum class State {
		MOVING,
		AT_REST,
		RECENTLY_AT_REST,
	}

	var state = State.MOVING
		private set

	// Instant that we entered the current state
	private var startTime = timeSource.markNow()

	// Rotation which could
	private var lastRotation = Quaternion.IDENTITY
	private var lastRotationTime = timeSource.markNow()

	/**
	 * Resets the detector
	 */
	fun reset() {
		val now = timeSource.markNow()

		state = State.MOVING
		startTime = now
		lastRotation = Quaternion.IDENTITY
		lastRotationTime = now
	}

	/**
	 * Provides a new rotation sample to the detector.
	 *
	 * @return whether the tracker is at rest
	 */
	fun update(rotation: Quaternion) {
		val now = timeSource.markNow()

		if (
			state == State.RECENTLY_AT_REST &&
			now > startTime.plus(enterMovingTime)
		) {
			state = State.MOVING
			startTime = now
			lastRotation = rotation
			lastRotationTime = now
		}

		when (state) {
			State.MOVING,
			State.RECENTLY_AT_REST,
			->
				if (Angle.absBetween(lastRotation, rotation) > enterRestRotation) {
					lastRotation = rotation
					lastRotationTime = now
				} else {
					// When we detect the tracker is at rest, use the current rotation as the
					// new start rotation for continuing to detect the tracker is at rest
					if (now > lastRotationTime.plus(enterRestTime)) {
						state = State.AT_REST
						startTime = now
						lastRotation = rotation
						lastRotationTime = now
					}
				}

			State.AT_REST ->
				if (Angle.absBetween(lastRotation, rotation) > exitRestRotation) {
					state = State.RECENTLY_AT_REST
					startTime = now
					lastRotation = rotation
					lastRotationTime = now
				}
		}
	}
}
