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
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** Concave triangle mesh abstract class. Use {@link BvhTriangleMeshShape} as concrete implementation.
 * 
 * @author jezek2 */
public abstract class TriangleMeshShape extends ConcaveShape {

	protected final Vector3 localAabbMin = new Vector3();
	protected final Vector3 localAabbMax = new Vector3();
	protected StridingMeshInterface meshInterface;

	/** TriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class. Don't use
	 * btTriangleMeshShape but use btBvhTriangleMeshShape instead! */
	protected TriangleMeshShape (StridingMeshInterface meshInterface) {
		this.meshInterface = meshInterface;

		// JAVA NOTE: moved to BvhTriangleMeshShape
		// recalcLocalAabb();
	}

	public Vector3 localGetSupportingVertex (Vector3 vec, Vector3 out) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		Vector3 supportVertex = out;

		Transform ident = stack.allocTransform();
		ident.setIdentity();

		SupportVertexCallback supportCallback = new SupportVertexCallback(vec, ident);

		Vector3 aabbMax = stack.allocVector3();
		aabbMax.set(1e30f, 1e30f, 1e30f);
		tmp.set(aabbMax).scl(-1);

		processAllTriangles(supportCallback, tmp, aabbMax);

		supportCallback.getSupportVertexLocal(supportVertex);
		stack.leave();
		return out;
	}

	public Vector3 localGetSupportingVertexWithoutMargin (Vector3 vec, Vector3 out) {
		assert (false);
		return localGetSupportingVertex(vec, out);
	}

	public void recalcLocalAabb () {
		Stack stack = Stack.enter();
		for (int i = 0; i < 3; i++) {
			Vector3 vec = stack.allocVector3();
			vec.set(0f, 0f, 0f);
			VectorUtil.setCoord(vec, i, 1f);
			Vector3 tmp = localGetSupportingVertex(vec, stack.allocVector3());
			VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + collisionMargin);
			VectorUtil.setCoord(vec, i, -1f);
			localGetSupportingVertex(vec, tmp);
			VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - collisionMargin);
		}
		stack.leave();
	}

	@Override
	public void getAabb (Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		Vector3 localHalfExtents = stack.allocVector3();
		localHalfExtents.set(localAabbMax).sub(localAabbMin);
		localHalfExtents.scl(0.5f);

		Vector3 localCenter = stack.allocVector3();
		localCenter.set(localAabbMax).add(localAabbMin);
		localCenter.scl(0.5f);

		Matrix3 abs_b = stack.alloc(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = stack.alloc(localCenter);
		trans.transform(center);

		Vector3 extent = stack.allocVector3();
		MatrixUtil.getRow(abs_b, 0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		Vector3 margin = stack.allocVector3();
		margin.set(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.set(center).sub(extent);
		aabbMax.set(center).add(extent);
		stack.leave();
	}

	@Override
	public void processAllTriangles (TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
		FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);

		meshInterface.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
	}

	@Override
	public void calculateLocalInertia (float mass, Vector3 inertia) {
		// moving concave objects not supported
		assert (false);
		inertia.set(0f, 0f, 0f);
	}

	@Override
	public void setLocalScaling (Vector3 scaling) {
		meshInterface.setScaling(scaling);
		recalcLocalAabb();
	}

	@Override
	public Vector3 getLocalScaling (Vector3 out) {
		return meshInterface.getScaling(out);
	}

	public StridingMeshInterface getMeshInterface () {
		return meshInterface;
	}

	public Vector3 getLocalAabbMin (Vector3 out) {
		out.set(localAabbMin);
		return out;
	}

	public Vector3 getLocalAabbMax (Vector3 out) {
		out.set(localAabbMax);
		return out;
	}

	@Override
	public String getName () {
		return "TRIANGLEMESH";
	}

	// //////////////////////////////////////////////////////////////////////////

	private class SupportVertexCallback extends TriangleCallback {
		private final Vector3 supportVertexLocal = new Vector3(0f, 0f, 0f);
		public final Transform worldTrans = new Transform();
		public float maxDot = -1e30f;
		public final Vector3 supportVecLocal = new Vector3();

		public SupportVertexCallback (Vector3 supportVecWorld, Transform trans) {
			this.worldTrans.set(trans);
			MatrixUtil.transposeTransform(supportVecLocal, supportVecWorld, worldTrans.basis);
		}

		public void processTriangle (Vector3[] triangle, int partId, int triangleIndex) {
			for (int i = 0; i < 3; i++) {
				float dot = supportVecLocal.dot(triangle[i]);
				if (dot > maxDot) {
					maxDot = dot;
					supportVertexLocal.set(triangle[i]);
				}
			}
		}

		public Vector3 getSupportVertexWorldSpace (Vector3 out) {
			out.set(supportVertexLocal);
			worldTrans.transform(out);
			return out;
		}

		public Vector3 getSupportVertexLocal (Vector3 out) {
			out.set(supportVertexLocal);
			return out;
		}
	}

	private static class FilteredCallback extends InternalTriangleIndexCallback {
		public TriangleCallback callback;
		public final Vector3 aabbMin = new Vector3();
		public final Vector3 aabbMax = new Vector3();

		public FilteredCallback (TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
			this.callback = callback;
			this.aabbMin.set(aabbMin);
			this.aabbMax.set(aabbMax);
		}

		public void internalProcessTriangleIndex (Vector3[] triangle, int partId, int triangleIndex) {
			if (AabbUtil2.testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) {
				// check aabb in triangle-space, before doing this
				callback.processTriangle(triangle, partId, triangleIndex);
			}
		}
	}

}
