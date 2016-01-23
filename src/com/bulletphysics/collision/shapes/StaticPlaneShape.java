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
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** StaticPlaneShape simulates an infinite non-moving (static) collision plane.
 * 
 * @author jezek2 */
public class StaticPlaneShape extends ConcaveShape {

	protected final Vector3 localAabbMin = new Vector3();
	protected final Vector3 localAabbMax = new Vector3();

	protected final Vector3 planeNormal = new Vector3();
	protected float planeConstant;
	protected final Vector3 localScaling = new Vector3(0f, 0f, 0f);

	public StaticPlaneShape (Vector3 planeNormal, float planeConstant) {
		this.planeNormal.set(planeNormal).nor();
		this.planeConstant = planeConstant;
	}

	public Vector3 getPlaneNormal (Vector3 out) {
		out.set(planeNormal);
		return out;
	}

	public float getPlaneConstant () {
		return planeConstant;
	}

	@Override
	public void processAllTriangles (TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		Vector3 halfExtents = stack.allocVector3();
		halfExtents.set(aabbMax).set(aabbMin);
		halfExtents.scl(0.5f);

		float radius = halfExtents.len();
		Vector3 center = stack.allocVector3();
		center.set(aabbMax).add(aabbMin);
		center.scl(0.5f);

		// this is where the triangles are generated, given AABB and plane equation (normal/constant)

		Vector3 tangentDir0 = stack.allocVector3(), tangentDir1 = stack.allocVector3();

		// tangentDir0/tangentDir1 can be precalculated
		TransformUtil.planeSpace1(planeNormal, tangentDir0, tangentDir1);

		Vector3 supVertex0 = stack.allocVector3(), supVertex1 = stack.allocVector3();

		Vector3 projectedCenter = stack.allocVector3();
		tmp.set(planeNormal).scl(planeNormal.dot(center) - planeConstant);
		projectedCenter.set(center).sub(tmp);

		Vector3[] triangle = new Vector3[] {stack.allocVector3(), stack.allocVector3(), stack.allocVector3()};

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		VectorUtil.add(triangle[0], projectedCenter, tmp1, tmp2);

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		tmp.set(tmp1).sub(tmp2);
		VectorUtil.add(triangle[1], projectedCenter, tmp);

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		tmp.set(tmp1).sub(tmp2);
		triangle[2].set(projectedCenter).sub(tmp);

		callback.processTriangle(triangle, 0, 0);

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		tmp.set(tmp1).sub(tmp2);
		triangle[0].set(projectedCenter).sub(tmp);

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		tmp.set(tmp1).add(tmp2);
		triangle[1].set(projectedCenter).sub(tmp);

		tmp1.set(tangentDir0).scl(radius);
		tmp2.set(tangentDir1).scl(radius);
		VectorUtil.add(triangle[2], projectedCenter, tmp1, tmp2);

		callback.processTriangle(triangle, 0, 1);
		stack.leave();
	}

	@Override
	public void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		aabbMin.set(-1e30f, -1e30f, -1e30f);
		aabbMax.set(1e30f, 1e30f, 1e30f);
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.STATIC_PLANE_PROXYTYPE;
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
		// moving concave objects not supported
		inertia.set(0f, 0f, 0f);
	}

	@Override
	public String getName () {
		return "STATICPLANE";
	}

}
