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

package com.bulletphysics.collision.shapes;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** CylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y
 * axis. {@link CylinderShapeX} is aligned with the X axis and {@link CylinderShapeZ} around the Z axis.
 * 
 * @author jezek2 */
public class CylinderShape extends BoxShape {

	protected int upAxis;

	public CylinderShape (Vector3 halfExtents) {
		super(halfExtents);
		upAxis = 1;
		recalcLocalAabb();
	}

	protected CylinderShape (Vector3 halfExtents, boolean unused) {
		super(halfExtents);
	}

	@Override
	public void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		_PolyhedralConvexShape_getAabb(t, aabbMin, aabbMax);
	}

	protected Vector3 cylinderLocalSupportX (Vector3 halfExtents, Vector3 v, Vector3 out) {
		return cylinderLocalSupport(halfExtents, v, 0, 1, 0, 2, out);
	}

	protected Vector3 cylinderLocalSupportY (Vector3 halfExtents, Vector3 v, Vector3 out) {
		return cylinderLocalSupport(halfExtents, v, 1, 0, 1, 2, out);
	}

	protected Vector3 cylinderLocalSupportZ (Vector3 halfExtents, Vector3 v, Vector3 out) {
		return cylinderLocalSupport(halfExtents, v, 2, 0, 2, 1, out);
	}

	private Vector3 cylinderLocalSupport (Vector3 halfExtents, Vector3 v, int cylinderUpAxis, int XX, int YY, int ZZ,
		Vector3 out) {
		// mapping depends on how cylinder local orientation is
		// extents of the cylinder is: X,Y is for radius, and Z for height

		float radius = VectorUtil.getCoord(halfExtents, XX);
		float halfHeight = VectorUtil.getCoord(halfExtents, cylinderUpAxis);

		float d;

		float s = (float)Math
			.sqrt(VectorUtil.getCoord(v, XX) * VectorUtil.getCoord(v, XX) + VectorUtil.getCoord(v, ZZ) * VectorUtil.getCoord(v, ZZ));
		if (s != 0f) {
			d = radius / s;
			VectorUtil.setCoord(out, XX, VectorUtil.getCoord(v, XX) * d);
			VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0f ? -halfHeight : halfHeight);
			VectorUtil.setCoord(out, ZZ, VectorUtil.getCoord(v, ZZ) * d);
			return out;
		} else {
			VectorUtil.setCoord(out, XX, radius);
			VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0f ? -halfHeight : halfHeight);
			VectorUtil.setCoord(out, ZZ, 0f);
			return out;
		}
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin (Vector3 vec, Vector3 out) {
		Stack stack = Stack.enter();
		Vector3 result = cylinderLocalSupportY(getHalfExtentsWithoutMargin(stack.allocVector3()), vec, out);
		stack.leave();
		return result;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin (Vector3[] vectors, Vector3[] supportVerticesOut,
		int numVectors) {
		Stack stack = Stack.enter();
		for (int i = 0; i < numVectors; i++) {
			cylinderLocalSupportY(getHalfExtentsWithoutMargin(stack.allocVector3()), vectors[i], supportVerticesOut[i]);
		}
		stack.leave();
	}

	@Override
	public Vector3 localGetSupportingVertex (Vector3 vec, Vector3 out) {
		Vector3 supVertex = out;
		localGetSupportingVertexWithoutMargin(vec, supVertex);

		if (getMargin() != 0f) {
			Stack stack = Stack.enter();
			Vector3 vecnorm = stack.alloc(vec);
			if (vecnorm.len2() < (BulletGlobals.SIMD_EPSILON * BulletGlobals.SIMD_EPSILON)) {
				vecnorm.set(-1f, -1f, -1f);
			}
			vecnorm.nor();
			vecnorm.scl(getMargin());
			supVertex.add(vecnorm);
			stack.leave();
		}
		return out;
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.CYLINDER_SHAPE_PROXYTYPE;
	}

	public int getUpAxis () {
		return upAxis;
	}

	public float getRadius () {
		Stack stack = Stack.enter();
		float result = getHalfExtentsWithMargin(stack.allocVector3()).x;
		stack.leave();
		return result;
	}

	@Override
	public String getName () {
		return "CylinderY";
	}

}
