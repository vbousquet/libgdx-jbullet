/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.linearmath;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.util.Stack;

/** Utility functions for axis aligned bounding boxes (AABB).
 * 
 * @author jezek2 */
public class AabbUtil2 {

	public static void aabbExpand (Vector3 aabbMin, Vector3 aabbMax, Vector3 expansionMin, Vector3 expansionMax) {
		aabbMin.add(expansionMin);
		aabbMax.add(expansionMax);
	}

	public static int outcode (Vector3 p, Vector3 halfExtent) {
		return (p.x < -halfExtent.x ? 0x01 : 0x0) | (p.x > halfExtent.x ? 0x08 : 0x0) | (p.y < -halfExtent.y ? 0x02 : 0x0)
			| (p.y > halfExtent.y ? 0x10 : 0x0) | (p.z < -halfExtent.z ? 0x4 : 0x0) | (p.z > halfExtent.z ? 0x20 : 0x0);
	}

	public static boolean rayAabb (Vector3 rayFrom, Vector3 rayTo, Vector3 aabbMin, Vector3 aabbMax, float[] param,
		Vector3 normal) {
		Stack stack = Stack.enter();
		Vector3 aabbHalfExtent = stack.allocVector3();
		Vector3 aabbCenter = stack.allocVector3();
		Vector3 source = stack.allocVector3();
		Vector3 target = stack.allocVector3();
		Vector3 r = stack.allocVector3();
		Vector3 hitNormal = stack.allocVector3();

		aabbHalfExtent.set(aabbMax).sub(aabbMin);
		aabbHalfExtent.scl(0.5f);

		aabbCenter.set(aabbMax).add(aabbMin);
		aabbCenter.scl(0.5f);

		source.set(rayFrom).sub(aabbCenter);
		target.set(rayTo).sub(aabbCenter);

		int sourceOutcode = outcode(source, aabbHalfExtent);
		int targetOutcode = outcode(target, aabbHalfExtent);
		if ((sourceOutcode & targetOutcode) == 0x0) {
			float lambda_enter = 0f;
			float lambda_exit = param[0];
			r.set(target).sub(source);

			float normSign = 1f;
			hitNormal.set(0f, 0f, 0f);
			int bit = 1;

			for (int j = 0; j < 2; j++) {
				for (int i = 0; i != 3; ++i) {
					if ((sourceOutcode & bit) != 0) {
						float lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign)
							/ VectorUtil.getCoord(r, i);
						if (lambda_enter <= lambda) {
							lambda_enter = lambda;
							hitNormal.set(0f, 0f, 0f);
							VectorUtil.setCoord(hitNormal, i, normSign);
						}
					} else if ((targetOutcode & bit) != 0) {
						float lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign)
							/ VectorUtil.getCoord(r, i);
						// btSetMin(lambda_exit, lambda);
						lambda_exit = Math.min(lambda_exit, lambda);
					}
					bit <<= 1;
				}
				normSign = -1f;
			}
			if (lambda_enter <= lambda_exit) {
				param[0] = lambda_enter;
				normal.set(hitNormal);
				stack.leave();
				return true;
			}
		}
		stack.leave();
		return false;
	}

	/** Conservative test for overlap between two AABBs. */
	public static boolean testAabbAgainstAabb2 (Vector3 aabbMin1, Vector3 aabbMax1, Vector3 aabbMin2, Vector3 aabbMax2) {
		boolean overlap = true;
		overlap = (aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x) ? false : overlap;
		overlap = (aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z) ? false : overlap;
		overlap = (aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y) ? false : overlap;
		return overlap;
	}

	/** Conservative test for overlap between triangle and AABB. */
	public static boolean testTriangleAgainstAabb2 (Vector3[] vertices, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 p1 = vertices[0];
		Vector3 p2 = vertices[1];
		Vector3 p3 = vertices[2];

		if (Math.min(Math.min(p1.x, p2.x), p3.x) > aabbMax.x) return false;
		if (Math.max(Math.max(p1.x, p2.x), p3.x) < aabbMin.x) return false;

		if (Math.min(Math.min(p1.z, p2.z), p3.z) > aabbMax.z) return false;
		if (Math.max(Math.max(p1.z, p2.z), p3.z) < aabbMin.z) return false;

		if (Math.min(Math.min(p1.y, p2.y), p3.y) > aabbMax.y) return false;
		if (Math.max(Math.max(p1.y, p2.y), p3.y) < aabbMin.y) return false;

		return true;
	}

	public static void transformAabb (Vector3 halfExtents, float margin, Transform t, Vector3 aabbMinOut, Vector3 aabbMaxOut) {
		Stack stack = Stack.enter();
		Vector3 halfExtentsWithMargin = stack.allocVector3();
		halfExtentsWithMargin.x = halfExtents.x + margin;
		halfExtentsWithMargin.y = halfExtents.y + margin;
		halfExtentsWithMargin.z = halfExtents.z + margin;

		Matrix3 abs_b = stack.alloc(t.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 tmp = stack.allocVector3();

		Vector3 center = stack.alloc(t.origin);
		Vector3 extent = stack.allocVector3();
		tmp.x = abs_b.val[Matrix3.M00];
		tmp.y = abs_b.val[Matrix3.M01];
		tmp.z = abs_b.val[Matrix3.M02];
		extent.x = tmp.dot(halfExtentsWithMargin);
		tmp.x = abs_b.val[Matrix3.M10];
		tmp.y = abs_b.val[Matrix3.M11];
		tmp.z = abs_b.val[Matrix3.M12];
		extent.y = tmp.dot(halfExtentsWithMargin);
		tmp.x = abs_b.val[Matrix3.M20];
		tmp.y = abs_b.val[Matrix3.M21];
		tmp.z = abs_b.val[Matrix3.M22];
		extent.z = tmp.dot(halfExtentsWithMargin);

		aabbMinOut.set(center).sub(extent);
		aabbMaxOut.set(center).add(extent);
		stack.leave();
	}

	public static void transformAabb (Vector3 localAabbMin, Vector3 localAabbMax, float margin, Transform trans,
		Vector3 aabbMinOut, Vector3 aabbMaxOut) {
		assert (localAabbMin.x <= localAabbMax.x);
		assert (localAabbMin.y <= localAabbMax.y);
		assert (localAabbMin.z <= localAabbMax.z);

		Stack stack = Stack.enter();
		Vector3 localHalfExtents = stack.allocVector3();
		localHalfExtents.set(localAabbMax).sub(localAabbMin);
		localHalfExtents.scl(0.5f);

		localHalfExtents.x += margin;
		localHalfExtents.y += margin;
		localHalfExtents.z += margin;

		Vector3 localCenter = stack.allocVector3();
		localCenter.set(localAabbMax).add(localAabbMin);
		localCenter.scl(0.5f);

		Matrix3 abs_b = stack.alloc(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = stack.alloc(localCenter);
		trans.transform(center);

		Vector3 extent = stack.allocVector3();
		Vector3 tmp = stack.allocVector3();

		tmp.x = abs_b.val[Matrix3.M00];
		tmp.y = abs_b.val[Matrix3.M01];
		tmp.z = abs_b.val[Matrix3.M02];
		extent.x = tmp.dot(localHalfExtents);
		tmp.x = abs_b.val[Matrix3.M10];
		tmp.y = abs_b.val[Matrix3.M11];
		tmp.z = abs_b.val[Matrix3.M12];
		extent.y = tmp.dot(localHalfExtents);
		tmp.x = abs_b.val[Matrix3.M20];
		tmp.y = abs_b.val[Matrix3.M21];
		tmp.z = abs_b.val[Matrix3.M22];
		extent.z = tmp.dot(localHalfExtents);

		aabbMinOut.set(center).sub(extent);
		aabbMaxOut.set(center).add(extent);
		stack.leave();
	}

}
