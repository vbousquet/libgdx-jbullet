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
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

// JAVA NOTE: ScaledBvhTriangleMeshShape from 2.73 SP1

/** The ScaledBvhTriangleMeshShape allows to instance a scaled version of an existing {@link BvhTriangleMeshShape}. Note that each
 * {@link BvhTriangleMeshShape} still can have its own local scaling, independent from this ScaledBvhTriangleMeshShape
 * 'localScaling'.
 *
 * @author jezek2 */
public class ScaledBvhTriangleMeshShape extends ConcaveShape {

	protected final Vector3 localScaling = new Vector3();
	protected BvhTriangleMeshShape bvhTriMeshShape;

	public ScaledBvhTriangleMeshShape (BvhTriangleMeshShape childShape, Vector3 localScaling) {
		this.localScaling.set(localScaling);
		this.bvhTriMeshShape = childShape;
	}

	public BvhTriangleMeshShape getChildShape () {
		return bvhTriMeshShape;
	}

	@Override
	public void processAllTriangles (TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		ScaledTriangleCallback scaledCallback = new ScaledTriangleCallback(callback, localScaling);

		Vector3 invLocalScaling = stack.allocVector3();
		invLocalScaling.set(1.f / localScaling.x, 1.f / localScaling.y, 1.f / localScaling.z);

		Vector3 scaledAabbMin = stack.allocVector3();
		Vector3 scaledAabbMax = stack.allocVector3();

		// support negative scaling
		scaledAabbMin.x = localScaling.x >= 0f ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMin.y = localScaling.y >= 0f ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMin.z = localScaling.z >= 0f ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		scaledAabbMax.x = localScaling.x <= 0f ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMax.y = localScaling.y <= 0f ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMax.z = localScaling.z <= 0f ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		bvhTriMeshShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax);
		stack.leave();
	}

	@Override
	public void getAabb (Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 localAabbMin = bvhTriMeshShape.getLocalAabbMin(stack.allocVector3());
		Vector3 localAabbMax = bvhTriMeshShape.getLocalAabbMax(stack.allocVector3());

		Vector3 tmpLocalAabbMin = stack.allocVector3();
		Vector3 tmpLocalAabbMax = stack.allocVector3();
		VectorUtil.mul(tmpLocalAabbMin, localAabbMin, localScaling);
		VectorUtil.mul(tmpLocalAabbMax, localAabbMax, localScaling);

		localAabbMin.x = (localScaling.x >= 0f) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMin.y = (localScaling.y >= 0f) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMin.z = (localScaling.z >= 0f) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;
		localAabbMax.x = (localScaling.x <= 0f) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMax.y = (localScaling.y <= 0f) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMax.z = (localScaling.z <= 0f) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;

		Vector3 localHalfExtents = stack.allocVector3();
		localHalfExtents.set(localAabbMax).sub(localAabbMin);
		localHalfExtents.scl(0.5f);

		float margin = bvhTriMeshShape.getMargin();
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
		MatrixUtil.getRow(abs_b, 0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		aabbMin.set(center).sub(extent);
		aabbMax.set(center).add(extent);
		stack.leave();
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	}

	@Override
	public void setLocalScaling (Vector3 scaling) {
		localScaling.set(scaling);
	}

	@Override
	public Vector3 getLocalScaling (Vector3 out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void calculateLocalInertia (float mass, Vector3 inertia) {
	}

	@Override
	public String getName () {
		return "SCALEDBVHTRIANGLEMESH";
	}

	// //////////////////////////////////////////////////////////////////////////

	private static class ScaledTriangleCallback extends TriangleCallback {
		private TriangleCallback originalCallback;
		private Vector3 localScaling;
		private Vector3[] newTriangle = new Vector3[3];

		public ScaledTriangleCallback (TriangleCallback originalCallback, Vector3 localScaling) {
			this.originalCallback = originalCallback;
			this.localScaling = localScaling;

			for (int i = 0; i < newTriangle.length; i++) {
				newTriangle[i] = new Vector3();
			}
		}

		public void processTriangle (Vector3[] triangle, int partId, int triangleIndex) {
			VectorUtil.mul(newTriangle[0], triangle[0], localScaling);
			VectorUtil.mul(newTriangle[1], triangle[1], localScaling);
			VectorUtil.mul(newTriangle[2], triangle[2], localScaling);
			originalCallback.processTriangle(newTriangle, partId, triangleIndex);
		}
	}

}
