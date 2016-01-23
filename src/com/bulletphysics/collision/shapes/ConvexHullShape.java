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
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;

/** ConvexHullShape implements an implicit convex hull of an array of vertices. Bullet provides a general and fast collision
 * detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
 * 
 * @author jezek2 */
public class ConvexHullShape extends PolyhedralConvexShape {

	private final ObjectArrayList<Vector3> points = new ObjectArrayList<Vector3>();

	/** TODO: This constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive float (x,y,z),
	 * the striding defines the number of bytes between each point, in memory. It is easier to not pass any points in the
	 * constructor, and just add one point at a time, using addPoint. ConvexHullShape make an internal copy of the points. */
	// TODO: make better constuctors (ByteBuffer, etc.)
	public ConvexHullShape (ObjectArrayList<Vector3> points) {
		// JAVA NOTE: rewritten

		for (int i = 0; i < points.size(); i++) {
			this.points.add(new Vector3(points.getQuick(i)));
		}

		recalcLocalAabb();
	}

	@Override
	public void setLocalScaling (Vector3 scaling) {
		localScaling.set(scaling);
		recalcLocalAabb();
	}

	public void addPoint (Vector3 point) {
		points.add(new Vector3(point));
		recalcLocalAabb();
	}

	public ObjectArrayList<Vector3> getPoints () {
		return points;
	}

	public int getNumPoints () {
		return points.size();
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin (Vector3 vec0, Vector3 out) {
		Vector3 supVec = out;
		supVec.set(0f, 0f, 0f);
		float newDot, maxDot = -1e30f;
		Stack stack = Stack.enter();

		Vector3 vec = stack.alloc(vec0);
		float lenSqr = vec.len2();
		if (lenSqr < 0.0001f) {
			vec.set(1f, 0f, 0f);
		} else {
			float rlen = 1f / (float)Math.sqrt(lenSqr);
			vec.scl(rlen);
		}

		Vector3 vtx = stack.allocVector3();
		for (int i = 0; i < points.size(); i++) {
			VectorUtil.mul(vtx, points.getQuick(i), localScaling);

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
	public void batchedUnitVectorGetSupportingVertexWithoutMargin (Vector3[] vectors, Vector3[] supportVerticesOut,
		int numVectors) {
		float newDot;

		// JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
		// TODO: optimize it
		float[] wcoords = new float[numVectors];

		// use 'w' component of supportVerticesOut?
		{
			for (int i = 0; i < numVectors; i++) {
				// supportVerticesOut[i][3] = btScalar(-1e30);
				wcoords[i] = -1e30f;
			}
		}
		Stack stack = Stack.enter();
		Vector3 vtx = stack.allocVector3();
		for (int i = 0; i < points.size(); i++) {
			VectorUtil.mul(vtx, points.getQuick(i), localScaling);

			for (int j = 0; j < numVectors; j++) {
				Vector3 vec = vectors[j];

				newDot = vec.dot(vtx);
				// if (newDot > supportVerticesOut[j][3])
				if (newDot > wcoords[j]) {
					// WARNING: don't swap next lines, the w component would get overwritten!
					supportVerticesOut[j].set(vtx);
					// supportVerticesOut[j][3] = newDot;
					wcoords[j] = newDot;
				}
			}
		}
		stack.leave();
	}

	@Override
	public Vector3 localGetSupportingVertex (Vector3 vec, Vector3 out) {
		Vector3 supVertex = localGetSupportingVertexWithoutMargin(vec, out);

		if (getMargin() != 0f) {
			Stack stack = Stack.enter();
			Vector3 vecnorm = stack.alloc(vec);
			if (vecnorm.len2() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
				vecnorm.set(-1f, -1f, -1f);
			}
			vecnorm.nor();
			vecnorm.scl(getMargin());
			supVertex.add(vecnorm);
			stack.leave();
		}
		return out;
	}

	/** Currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection. Please note
	 * that you can debug-draw ConvexHullShape with the Raytracer Demo. */
	@Override
	public int getNumVertices () {
		return points.size();
	}

	@Override
	public int getNumEdges () {
		return points.size();
	}

	@Override
	public void getEdge (int i, Vector3 pa, Vector3 pb) {
		int index0 = i % points.size();
		int index1 = (i + 1) % points.size();
		VectorUtil.mul(pa, points.getQuick(index0), localScaling);
		VectorUtil.mul(pb, points.getQuick(index1), localScaling);
	}

	@Override
	public void getVertex (int i, Vector3 vtx) {
		VectorUtil.mul(vtx, points.getQuick(i), localScaling);
	}

	@Override
	public int getNumPlanes () {
		return 0;
	}

	@Override
	public void getPlane (Vector3 planeNormal, Vector3 planeSupport, int i) {
		assert false;
	}

	@Override
	public boolean isInside (Vector3 pt, float tolerance) {
		assert false;
		return false;
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.CONVEX_HULL_SHAPE_PROXYTYPE;
	}

	@Override
	public String getName () {
		return "Convex";
	}

}
