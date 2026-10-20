package dev.slimevr.unit

import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.stayaligned.trackers.RestDetector
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TestTimeSource

/**
 * Tests the rest hysteresis that feeds Stay Aligned's drift estimate.
 */
class RestDetectorTests {

	private fun makeDetector(source: TestTimeSource) = RestDetector(
		enterRestRotation = Angle.ofDeg(2.0f),
		exitRestRotation = Angle.ofDeg(10.0f),
		enterRestTime = 1.seconds,
		enterMovingTime = 3.seconds,
		timeSource = source,
	)

	private fun RestDetector.step(source: TestTimeSource, rotation: Quaternion, ms: Long) {
		source += ms.milliseconds
		update(rotation)
	}

	@Test
	fun `slow drift keeps the tracker at rest`() {
		val source = TestTimeSource()
		val detector = makeDetector(source)

		detector.update(Quaternion.IDENTITY)
		detector.step(source, Quaternion.IDENTITY, 1_100)
		Assertions.assertEquals(RestDetector.State.AT_REST, detector.state)

		// 3 deg/min for two minutes is 6 deg, inside the exit range.
		var time = 0.0f
		while (time < 120.0f) {
			source += 1.seconds
			time += 1.0f
			val rotation = Quaternion.rotationAroundYAxis(
				Angle.ofDeg(3.0f).toRad() / 60.0f * time,
			)
			detector.update(rotation)
		}
		Assertions.assertEquals(RestDetector.State.AT_REST, detector.state)
	}

	@Test
	fun `drift past the exit range unlocks the tracker`() {
		val source = TestTimeSource()
		val detector = makeDetector(source)

		detector.update(Quaternion.IDENTITY)
		detector.step(source, Quaternion.IDENTITY, 1_100)

		// 3 deg/min for four minutes is 12 deg.
		source += 240.seconds
		detector.update(Quaternion.rotationAroundYAxis(Angle.ofDeg(12.0f).toRad()))
		Assertions.assertEquals(RestDetector.State.RECENTLY_AT_REST, detector.state)
	}

	@Test
	fun `real motion leaves rest immediately`() {
		val source = TestTimeSource()
		val detector = makeDetector(source)

		detector.update(Quaternion.IDENTITY)
		detector.step(source, Quaternion.IDENTITY, 1_100)
		Assertions.assertEquals(RestDetector.State.AT_REST, detector.state)

		source += 10.milliseconds
		detector.update(Quaternion.rotationAroundYAxis(Angle.ofDeg(30.0f).toRad()))
		Assertions.assertEquals(RestDetector.State.RECENTLY_AT_REST, detector.state)

		detector.step(
			source,
			Quaternion.rotationAroundYAxis(Angle.ofDeg(30.0f).toRad()),
			3_100,
		)
		Assertions.assertEquals(RestDetector.State.MOVING, detector.state)
	}

	@Test
	fun `small wobble does not unlock a resting tracker`() {
		val source = TestTimeSource()
		val detector = makeDetector(source)

		detector.update(Quaternion.IDENTITY)
		detector.step(source, Quaternion.IDENTITY, 1_100)

		for (i in 0 until 10) {
			source += 500.milliseconds
			val wobble = if (i % 2 == 0) 1.5f else -1.5f
			detector.update(Quaternion.rotationAroundYAxis(Angle.ofDeg(wobble).toRad()))
		}
		Assertions.assertEquals(RestDetector.State.AT_REST, detector.state)
	}
}
