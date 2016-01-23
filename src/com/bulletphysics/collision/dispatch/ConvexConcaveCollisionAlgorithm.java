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

package com.bulletphysics.collision.dispatch;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.narrowphase.SubsimplexConvexCast;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.ObjectPool;
import com.bulletphysics.util.Stack;
import com.bulletphysics.util.Supplier;

/** ConvexConcaveCollisionAlgorithm supports collision between convex shapes and (concave) trianges meshes.
 * 
 * @author jezek2 */
public class ConvexConcaveCollisionAlgorithm extends CollisionAlgorithm {

	private boolean isSwapped;
	private ConvexTriangleCallback btConvexTriangleCallback;

	public void init (CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, boolean isSwapped) {
		super.init(ci);
		this.isSwapped = isSwapped;
		this.btConvexTriangleCallback = new ConvexTriangleCallback(dispatcher, body0, body1, isSwapped);
	}

	@Override
	public void destroy () {
		btConvexTriangleCallback.destroy();
	}

	@Override
	public void processCollision (CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo,
		ManifoldResult resultOut) {
		CollisionObject convexBody = isSwapped ? body1 : body0;
		CollisionObject triBody = isSwapped ? body0 : body1;

		if (triBody.getCollisionShape().isConcave()) {
			CollisionObject triOb = triBody;
			ConcaveShape concaveShape = (ConcaveShape)triOb.getCollisionShape();

			if (convexBody.getCollisionShape().isConvex()) {
				float collisionMarginTriangle = concaveShape.getMargin();

				resultOut.setPersistentManifold(btConvexTriangleCallback.manifoldPtr);
				btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);

				// Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
				// m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

				btConvexTriangleCallback.manifoldPtr.setBodies(convexBody, triBody);

				Stack stack = Stack.enter();
				concaveShape.processAllTriangles(btConvexTriangleCallback, btConvexTriangleCallback.getAabbMin(stack.allocVector3()),
					btConvexTriangleCallback.getAabbMax(stack.allocVector3()));

				resultOut.refreshContactPoints();
				stack.leave();
			}
		}
	}

	@Override
	public float calculateTimeOfImpact (CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo,
		ManifoldResult resultOut) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();

		CollisionObject convexbody = isSwapped ? body1 : body0;
		CollisionObject triBody = isSwapped ? body0 : body1;

		// quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

		// only perform CCD above a certain threshold, this prevents blocking on the long run
		// because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
		tmp.set(convexbody.getInterpolationWorldTransform(stack.allocTransform()).origin)
			.sub(convexbody.getWorldTransform(stack.allocTransform()).origin);
		float squareMot0 = tmp.len2();
		if (squareMot0 < convexbody.getCcdSquareMotionThreshold()) {
			stack.leave();
			return 1f;
		}

		Transform tmpTrans = stack.allocTransform();

		// const btVector3& from = convexbody->m_worldTransform.getOrigin();
		// btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
		// todo: only do if the motion exceeds the 'radius'

		Transform triInv = triBody.getWorldTransform(stack.allocTransform());
		triInv.inverse();

		Transform convexFromLocal = stack.allocTransform();
		convexFromLocal.mul(triInv, convexbody.getWorldTransform(tmpTrans));

		Transform convexToLocal = stack.allocTransform();
		convexToLocal.mul(triInv, convexbody.getInterpolationWorldTransform(tmpTrans));

		if (triBody.getCollisionShape().isConcave()) {
			Vector3 rayAabbMin = stack.alloc(convexFromLocal.origin);
			VectorUtil.setMin(rayAabbMin, convexToLocal.origin);

			Vector3 rayAabbMax = stack.alloc(convexFromLocal.origin);
			VectorUtil.setMax(rayAabbMax, convexToLocal.origin);

			float ccdRadius0 = convexbody.getCcdSweptSphereRadius();

			tmp.set(ccdRadius0, ccdRadius0, ccdRadius0);
			rayAabbMin.sub(tmp);
			rayAabbMax.add(tmp);

			float curHitFraction = 1f; // is this available?
			LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(convexFromLocal, convexToLocal,
				convexbody.getCcdSweptSphereRadius(), curHitFraction);

			raycastCallback.hitFraction = convexbody.getHitFraction();

			CollisionObject concavebody = triBody;

			ConcaveShape triangleMesh = (ConcaveShape)concavebody.getCollisionShape();

			if (triangleMesh != null) {
				triangleMesh.processAllTriangles(raycastCallback, rayAabbMin, rayAabbMax);
			}

			if (raycastCallback.hitFraction < convexbody.getHitFraction()) {
				convexbody.setHitFraction(raycastCallback.hitFraction);
				stack.leave();
				return raycastCallback.hitFraction;
			}
		}

		stack.leave();
		return 1f;
	}

	@Override
	public void getAllContactManifolds (ObjectArrayList<PersistentManifold> manifoldArray) {
		if (btConvexTriangleCallback.manifoldPtr != null) {
			manifoldArray.add(btConvexTriangleCallback.manifoldPtr);
		}
	}

	public void clearCache () {
		btConvexTriangleCallback.clearCache();
	}

	////////////////////////////////////////////////////////////////////////////

	private static class LocalTriangleSphereCastCallback extends TriangleCallback {
		public final Transform ccdSphereFromTrans = new Transform();
		public final Transform ccdSphereToTrans = new Transform();
		public final Transform meshTransform = new Transform();

		public float ccdSphereRadius;
		public float hitFraction;

		private final Transform ident = new Transform();

		public LocalTriangleSphereCastCallback (Transform from, Transform to, float ccdSphereRadius, float hitFraction) {
			this.ccdSphereFromTrans.set(from);
			this.ccdSphereToTrans.set(to);
			this.ccdSphereRadius = ccdSphereRadius;
			this.hitFraction = hitFraction;

			// JAVA NOTE: moved here from processTriangle
			ident.setIdentity();
		}

		public void processTriangle (Vector3[] triangle, int partId, int triangleIndex) {
			// do a swept sphere for now

			// btTransform ident;
			// ident.setIdentity();

			CastResult castResult = new CastResult();
			castResult.fraction = hitFraction;
			SphereShape pointShape = new SphereShape(ccdSphereRadius);
			TriangleShape triShape = new TriangleShape(triangle[0], triangle[1], triangle[2]);
			VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
			SubsimplexConvexCast convexCaster = new SubsimplexConvexCast(pointShape, triShape, simplexSolver);
			// GjkConvexCast convexCaster(&pointShape,convexShape,&simplexSolver);
			// ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			// local space?

			if (convexCaster.calcTimeOfImpact(ccdSphereFromTrans, ccdSphereToTrans, ident, ident, castResult)) {
				if (hitFraction > castResult.fraction) {
					hitFraction = castResult.fraction;
				}
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////

	public static class CreateFunc extends CollisionAlgorithmCreateFunc {
		private final ObjectPool<ConvexConcaveCollisionAlgorithm> pool = ObjectPool.get(ConvexConcaveCollisionAlgorithm.class,
			new Supplier<ConvexConcaveCollisionAlgorithm>() {
				@Override
				public ConvexConcaveCollisionAlgorithm get () {
					return new ConvexConcaveCollisionAlgorithm();
				}
			});

		@Override
		public CollisionAlgorithm createCollisionAlgorithm (CollisionAlgorithmConstructionInfo ci, CollisionObject body0,
			CollisionObject body1) {
			ConvexConcaveCollisionAlgorithm algo = pool.get();
			algo.init(ci, body0, body1, false);
			return algo;
		}

		@Override
		public void releaseCollisionAlgorithm (CollisionAlgorithm algo) {
			pool.release((ConvexConcaveCollisionAlgorithm)algo);
		}
	}

	public static class SwappedCreateFunc extends CollisionAlgorithmCreateFunc {
		private final ObjectPool<ConvexConcaveCollisionAlgorithm> pool = ObjectPool.get(ConvexConcaveCollisionAlgorithm.class,
			new Supplier<ConvexConcaveCollisionAlgorithm>() {
				@Override
				public ConvexConcaveCollisionAlgorithm get () {
					return new ConvexConcaveCollisionAlgorithm();
				}
			});

		@Override
		public CollisionAlgorithm createCollisionAlgorithm (CollisionAlgorithmConstructionInfo ci, CollisionObject body0,
			CollisionObject body1) {
			ConvexConcaveCollisionAlgorithm algo = pool.get();
			algo.init(ci, body0, body1, true);
			return algo;
		}

		@Override
		public void releaseCollisionAlgorithm (CollisionAlgorithm algo) {
			pool.release((ConvexConcaveCollisionAlgorithm)algo);
		}
	}

}
