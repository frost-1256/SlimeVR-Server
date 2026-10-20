package dev.slimevr.unit

import dev.slimevr.math.Quaternions
import dev.slimevr.tracking.trackers.udp.UDPUtils
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.sqrt

class QuaternionsTests {
	private fun length(q: Quaternion) = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)

	private fun assertSame(expected: Quaternion, actual: Quaternion, tolerance: Float = 1e-5f) {
		assertEquals(expected.w, actual.w, tolerance)
		assertEquals(expected.x, actual.x, tolerance)
		assertEquals(expected.y, actual.y, tolerance)
		assertEquals(expected.z, actual.z, tolerance)
	}

	@Test
	fun keepsUnitQuaternions() {
		val rotation = Quaternion(0.5f, 0.5f, 0.5f, 0.5f)
		assertSame(rotation, Quaternions.sanitize(rotation))
	}

	@Test
	fun normalizesScaledQuaternions() {
		val rotation = Quaternions.sanitize(Quaternion(2f, 0f, 0f, 2f))
		assertEquals(1f, length(rotation), 1e-5f)
		assertSame(Quaternion(0.70710677f, 0f, 0f, 0.70710677f), rotation)
	}

	@Test
	fun rejectsDegenerateInput() {
		assertSame(Quaternion.IDENTITY, Quaternions.sanitize(Quaternion(0f, 0f, 0f, 0f)))
		assertSame(
			Quaternion.IDENTITY,
			Quaternions.sanitize(Quaternion(0f, 1e-9f, 0f, 0f)),
		)
		assertSame(
			Quaternion.IDENTITY,
			Quaternions.sanitize(Quaternion(Float.NaN, 0f, 0f, 0f)),
		)
		assertSame(
			Quaternion.IDENTITY,
			Quaternions.sanitize(Quaternion(1f, Float.POSITIVE_INFINITY, 0f, 0f)),
		)
	}

	private fun bufferOf(vararg values: Float): ByteBuffer = ByteBuffer.allocate(values.size * Float.SIZE_BYTES)
		.order(ByteOrder.LITTLE_ENDIAN)
		.apply { values.forEach(::putFloat) }
		.flip()

	@Test
	fun udpParserNormalizesInsteadOfTrustingTheWire() {
		val rotation = UDPUtils.getSafeBufferQuaternion(bufferOf(0f, 0f, 0f, 4f))
		assertEquals(1f, length(rotation), 1e-5f)
		assertSame(Quaternion.IDENTITY, rotation)
	}

	@Test
	fun udpParserReplacesCorruptQuaternionWithIdentity() {
		assertSame(
			Quaternion.IDENTITY,
			UDPUtils.getSafeBufferQuaternion(
				bufferOf(Float.NaN, Float.NaN, Float.NaN, Float.NaN),
			),
		)
	}
}
