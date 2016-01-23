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

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** CapsuleShape represents a capsule around the Y axis, there is also the {@link CapsuleShapeX} aligned around the X axis and
 * {@link CapsuleShapeZ} around the Z axis.
 * <p>
 *
 * The total height is height+2*radius, so the height is just the height between the center of each "sphere" of the capsule caps.
 * <p>
 *
 * CapsuleShape is a convex hull of two spheres. The {@link MultiSphereShape} is a more general collision shape that takes the
 * convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
 * 
 * @author jezek2 */
public class CapsuleShape extends ConvexInternalShape {

	protected int upAxis;

	// only used for CapsuleShapeZ and CapsuleShapeX subclasses.
	CapsuleShape () {
	}

	public CapsuleShape (float radius, float height) {
		upAxis = 1;
		implicitShapeDimensions.set(radius, 0.5f * height, radius);
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin (Vector3 vec0, Vector3 out) {
		Stack stack = Stack.enter();
		Vector3 supVec = out;
		supVec.set(0f, 0f, 0f);

		float maxDot = -1e30f;

		Vector3 vec = stack.alloc(vec0);
		float lenSqr = vec.len2();
		if (lenSqr < 0.0001f) {
			vec.set(1f, 0f, 0f);
		} else {
			float rlen = 1f / (float)Math.sqrt(lenSqr);
			vec.scl(rlen);
		}

		Vector3 vtx = stack.allocVector3();
		float newDot;

		float radius = getRadius();

		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();
		Vector3 pos = stack.allocVector3();

		{
			pos.set(0f, 0f, 0f);
			VectorUtil.setCoord(pos, getUpAxis(), getHalfHeight());

			VectorUtil.mul(tmp1, vec, localScaling);
			tmp1.scl(radius);
			tmp2.set(vec).scl(getMargin());
			vtx.set(pos).add(tmp1);
			vtx.sub(tmp2);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec.set(vtx);
			}
		}
		{
			pos.set(0f, 0f, 0f);
			VectorUtil.setCoord(pos, getUpAxis(), -getHalfHeight());

			VectorUtil.mul(tmp1, vec, localScaling);
			tmp1.scl(radius);
			tmp2.set(vec).scl(getMargin());
			vtx.set(pos).add(tmp1);
			vtx.sub(tmp2);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec.set(vtx);
			}
		}
		stack.leave();
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin (Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
		// TODO: implement
		throw new UnsupportedOperationException("Not supported yet.");
	}

	@Override
	public void calculateLocalInertia (float mass, Vector3 inertia) {
		// as an approximation, take the inertia of the box that bounds the spheres

		Stack stack = Stack.enter();
		Transform ident = stack.allocTransform();
		ident.setIdentity();

		float radius = getRadius();

		Vector3 halfExtents = stack.allocVector3();
		halfExtents.set(radius, radius, radius);
		VectorUtil.setCoord(halfExtents, getUpAxis(), radius + getHalfHeight());

		float margin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

		float lx = 2f * (halfExtents.x + margin);
		float ly = 2f * (halfExtents.y + margin);
		float lz = 2f * (halfExtents.z + margin);
		float x2 = lx * lx;
		float y2 = ly * ly;
		float z2 = lz * lz;
		float scaledmass = mass * 0.08333333f;

		inertia.x = scaledmass * (y2 + z2);
		inertia.y = scaledmass * (x2 + z2);
		inertia.z = scaledmass * (x2 + y2);
		stack.leave();
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.CAPSULE_SHAPE_PROXYTYPE;
	}

	@Override
	public void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		Vector3 halfExtents = stack.allocVector3();
		halfExtents.set(getRadius(), getRadius(), getRadius());
		VectorUtil.setCoord(halfExtents, upAxis, getRadius() + getHalfHeight());

		halfExtents.x += getMargin();
		halfExtents.y += getMargin();
		halfExtents.z += getMargin();

		Matrix3 abs_b = stack.allocMatrix3();
		abs_b.set(t.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = t.origin;
		Vector3 extent = stack.allocVector3();

		MatrixUtil.getRow(abs_b, 0, tmp);
		extent.x = tmp.dot(halfExtents);
		MatrixUtil.getRow(abs_b, 1, tmp);
		extent.y = tmp.dot(halfExtents);
		MatrixUtil.getRow(abs_b, 2, tmp);
		extent.z = tmp.dot(halfExtents);

		aabbMin.set(center).sub(extent);
		aabbMax.set(center).add(extent);
		stack.leave();
	}

	@Override
	public String getName () {
		return "CapsuleShape";
	}

	public int getUpAxis () {
		return upAxis;
	}

	public float getRadius () {
		int radiusAxis = (upAxis + 2) % 3;
		return VectorUtil.getCoord(implicitShapeDimensions, radiusAxis);
	}

	public float getHalfHeight () {
		return VectorUtil.getCoord(implicitShapeDimensions, upAxis);
	}

}
