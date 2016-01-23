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

package com.bulletphysics.collision.narrowphase;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** @author jezek2 */
public abstract class TriangleRaycastCallback extends TriangleCallback {

	// protected final BulletStack stack = BulletStack.get();

	public final Vector3 from = new Vector3();
	public final Vector3 to = new Vector3();

	public float hitFraction;

	public TriangleRaycastCallback (Vector3 from, Vector3 to) {
		this.from.set(from);
		this.to.set(to);
		this.hitFraction = 1f;
	}

	public void processTriangle (Vector3[] triangle, int partId, int triangleIndex) {
		Stack stack = Stack.enter();
		Vector3 vert0 = triangle[0];
		Vector3 vert1 = triangle[1];
		Vector3 vert2 = triangle[2];

		Vector3 v10 = stack.allocVector3();
		v10.set(vert1).sub(vert0);

		Vector3 v20 = stack.allocVector3();
		v20.set(vert2).sub(vert0);

		Vector3 triangleNormal = stack.allocVector3();
		triangleNormal.set(v10).crs(v20);

		float dist = vert0.dot(triangleNormal);
		float dist_a = triangleNormal.dot(from);
		dist_a -= dist;
		float dist_b = triangleNormal.dot(to);
		dist_b -= dist;

		if (dist_a * dist_b >= 0f) {
			stack.leave();
			return; // same sign
		}

		float proj_length = dist_a - dist_b;
		float distance = (dist_a) / (proj_length);
		// Now we have the intersection point on the plane, we'll see if it's inside the triangle
		// Add an epsilon as a tolerance for the raycast,
		// in case the ray hits exacly on the edge of the triangle.
		// It must be scaled for the triangle size.

		if (distance < hitFraction) {
			float edge_tolerance = triangleNormal.len2();
			edge_tolerance *= -0.0001f;
			Vector3 point = new Vector3();
			VectorUtil.setInterpolate3(point, from, to, distance);
			{
				Vector3 v0p = stack.allocVector3();
				v0p.set(vert0).sub(point);
				Vector3 v1p = stack.allocVector3();
				v1p.set(vert1).sub(point);
				Vector3 cp0 = stack.allocVector3();
				cp0.set(v0p).crs(v1p);

				if (cp0.dot(triangleNormal) >= edge_tolerance) {
					Vector3 v2p = stack.allocVector3();
					v2p.set(vert2).sub(point);
					Vector3 cp1 = stack.allocVector3();
					cp1.set(v1p).crs(v2p);
					if (cp1.dot(triangleNormal) >= edge_tolerance) {
						Vector3 cp2 = stack.allocVector3();
						cp2.set(v2p).crs(v0p);

						if (cp2.dot(triangleNormal) >= edge_tolerance) {

							if (dist_a > 0f) {
								hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
							} else {
								Vector3 tmp = stack.allocVector3();
								tmp.set(triangleNormal).scl(-1);
								hitFraction = reportHit(tmp, distance, partId, triangleIndex);
							}
						}
					}
				}
			}
		}
		stack.leave();
	}

	public abstract float reportHit (Vector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex);

}
