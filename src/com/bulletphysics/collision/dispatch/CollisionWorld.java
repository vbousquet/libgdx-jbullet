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

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.broadphase.OverlappingPairCache;
import com.bulletphysics.collision.narrowphase.ConvexCast;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.narrowphase.GjkConvexCast;
import com.bulletphysics.collision.narrowphase.GjkEpaPenetrationDepthSolver;
import com.bulletphysics.collision.narrowphase.SubsimplexConvexCast;
import com.bulletphysics.collision.narrowphase.TriangleConvexcastCallback;
import com.bulletphysics.collision.narrowphase.TriangleRaycastCallback;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.collision.shapes.TriangleMeshShape;
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;

/** CollisionWorld is interface and container for the collision detection.
 * 
 * @author jezek2 */
public class CollisionWorld {

	// protected final BulletStack stack = BulletStack.get();

	protected ObjectArrayList<CollisionObject> collisionObjects = new ObjectArrayList<CollisionObject>();
	protected Dispatcher dispatcher1;
	protected DispatcherInfo dispatchInfo = new DispatcherInfo();
	// protected btStackAlloc* m_stackAlloc;
	protected BroadphaseInterface broadphasePairCache;
	protected IDebugDraw debugDrawer;

	/** This constructor doesn't own the dispatcher and paircache/broadphase. */
	public CollisionWorld (Dispatcher dispatcher, BroadphaseInterface broadphasePairCache,
		CollisionConfiguration collisionConfiguration) {
		this.dispatcher1 = dispatcher;
		this.broadphasePairCache = broadphasePairCache;
	}

	public void destroy () {
		// clean up remaining objects
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject collisionObject = collisionObjects.getQuick(i);

			BroadphaseProxy bp = collisionObject.getBroadphaseHandle();
			if (bp != null) {
				//
				// only clear the cached algorithms
				//
				getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, dispatcher1);
				getBroadphase().destroyProxy(bp, dispatcher1);
			}
		}
	}

	public void addCollisionObject (CollisionObject collisionObject) {
		addCollisionObject(collisionObject, CollisionFilterGroups.DEFAULT_FILTER, CollisionFilterGroups.ALL_FILTER);
	}

	public void addCollisionObject (CollisionObject collisionObject, short collisionFilterGroup, short collisionFilterMask) {
		// check that the object isn't already added
		assert (!collisionObjects.contains(collisionObject));

		Stack stack = Stack.enter();
		collisionObjects.add(collisionObject);

		// calculate new AABB
		// TODO: check if it's overwritten or not
		Transform trans = collisionObject.getWorldTransform(stack.allocTransform());

		Vector3 minAabb = stack.allocVector3();
		Vector3 maxAabb = stack.allocVector3();
		collisionObject.getCollisionShape().getAabb(trans, minAabb, maxAabb);

		BroadphaseNativeType type = collisionObject.getCollisionShape().getShapeType();
		collisionObject.setBroadphaseHandle(getBroadphase().createProxy(minAabb, maxAabb, type, collisionObject,
			collisionFilterGroup, collisionFilterMask, dispatcher1, null));
		stack.leave();
	}

	public void performDiscreteCollisionDetection () {
		BulletStats.pushProfile("performDiscreteCollisionDetection");
		try {
			// DispatcherInfo dispatchInfo = getDispatchInfo();

			updateAabbs();

			BulletStats.pushProfile("calculateOverlappingPairs");
			try {
				broadphasePairCache.calculateOverlappingPairs(dispatcher1);
			} finally {
				BulletStats.popProfile();
			}

			Dispatcher dispatcher = getDispatcher();
			{
				BulletStats.pushProfile("dispatchAllCollisionPairs");
				try {
					if (dispatcher != null) {
						dispatcher.dispatchAllCollisionPairs(broadphasePairCache.getOverlappingPairCache(), dispatchInfo, dispatcher1);
					}
				} finally {
					BulletStats.popProfile();
				}
			}
		} finally {
			BulletStats.popProfile();
		}
	}

	public void removeCollisionObject (CollisionObject collisionObject) {
		// bool removeFromBroadphase = false;

		{
			BroadphaseProxy bp = collisionObject.getBroadphaseHandle();
			if (bp != null) {
				//
				// only clear the cached algorithms
				//
				getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, dispatcher1);
				getBroadphase().destroyProxy(bp, dispatcher1);
				collisionObject.setBroadphaseHandle(null);
			}
		}

		// swapremove
		collisionObjects.remove(collisionObject);
	}

	public void setBroadphase (BroadphaseInterface pairCache) {
		broadphasePairCache = pairCache;
	}

	public BroadphaseInterface getBroadphase () {
		return broadphasePairCache;
	}

	public OverlappingPairCache getPairCache () {
		return broadphasePairCache.getOverlappingPairCache();
	}

	public Dispatcher getDispatcher () {
		return dispatcher1;
	}

	public DispatcherInfo getDispatchInfo () {
		return dispatchInfo;
	}

	private static boolean updateAabbs_reportMe = true;

	// JAVA NOTE: ported from 2.74, missing contact threshold stuff
	public void updateSingleAabb (CollisionObject colObj) {
		Stack stack = Stack.enter();
		Vector3 minAabb = stack.allocVector3(), maxAabb = stack.allocVector3();
		Vector3 tmp = stack.allocVector3();
		Transform tmpTrans = stack.allocTransform();

		colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
		// need to increase the aabb for contact thresholds
		Vector3 contactThreshold = stack.allocVector3();
		contactThreshold.set(BulletGlobals.getContactBreakingThreshold(), BulletGlobals.getContactBreakingThreshold(),
			BulletGlobals.getContactBreakingThreshold());
		minAabb.sub(contactThreshold);
		maxAabb.add(contactThreshold);

		BroadphaseInterface bp = broadphasePairCache;

		// moving objects should be moderately sized, probably something wrong if not
		tmp.set(maxAabb).sub(minAabb); // TODO: optimize
		if (colObj.isStaticObject() || (tmp.len2() < 1e12f)) {
			bp.setAabb(colObj.getBroadphaseHandle(), minAabb, maxAabb, dispatcher1);
		} else {
			// something went wrong, investigate
			// this assert is unwanted in 3D modelers (danger of loosing work)
			colObj.setActivationState(CollisionObject.DISABLE_SIMULATION);

			if (updateAabbs_reportMe && debugDrawer != null) {
				updateAabbs_reportMe = false;
				debugDrawer.reportErrorWarning("Overflow in AABB, object removed from simulation");
				debugDrawer.reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
				debugDrawer.reportErrorWarning("Please include above information, your Platform, version of OS.\n");
				debugDrawer.reportErrorWarning("Thanks.\n");
			}
		}
		stack.leave();
	}

	public void updateAabbs () {
		BulletStats.pushProfile("updateAabbs");
		try {
			for (int i = 0; i < collisionObjects.size(); i++) {
				CollisionObject colObj = collisionObjects.getQuick(i);

				// only update aabb of active objects
				if (colObj.isActive()) {
					updateSingleAabb(colObj);
				}
			}
		} finally {
			BulletStats.popProfile();
		}
	}

	public IDebugDraw getDebugDrawer () {
		return debugDrawer;
	}

	public void setDebugDrawer (IDebugDraw debugDrawer) {
		this.debugDrawer = debugDrawer;
	}

	public int getNumCollisionObjects () {
		return collisionObjects.size();
	}

	// TODO
	public static void rayTestSingle (Transform rayFromTrans, Transform rayToTrans, CollisionObject collisionObject,
		CollisionShape collisionShape, Transform colObjWorldTransform, RayResultCallback resultCallback) {
		Stack stack = Stack.enter();
		SphereShape pointShape = new SphereShape(0f);
		pointShape.setMargin(0f);
		ConvexShape castShape = pointShape;

		if (collisionShape.isConvex()) {
			CastResult castResult = new CastResult();
			castResult.fraction = resultCallback.closestHitFraction;

			ConvexShape convexShape = (ConvexShape)collisionShape;
			VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();

			// #define USE_SUBSIMPLEX_CONVEX_CAST 1
			// #ifdef USE_SUBSIMPLEX_CONVEX_CAST
			SubsimplexConvexCast convexCaster = new SubsimplexConvexCast(castShape, convexShape, simplexSolver);
			// #else
			// btGjkConvexCast convexCaster(castShape,convexShape,&simplexSolver);
			// btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);
			// #endif //#USE_SUBSIMPLEX_CONVEX_CAST

			if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform, colObjWorldTransform, castResult)) {
				// add hit
				if (castResult.normal.len2() > 0.0001f) {
					if (castResult.fraction < resultCallback.closestHitFraction) {
						// #ifdef USE_SUBSIMPLEX_CONVEX_CAST
						// rotate normal into worldspace
						castResult.normal.mul(rayFromTrans.basis);
						// #endif //USE_SUBSIMPLEX_CONVEX_CAST

						castResult.normal.nor();
						LocalRayResult localRayResult = new LocalRayResult(collisionObject, null, castResult.normal,
							castResult.fraction);

						boolean normalInWorldSpace = true;
						resultCallback.addSingleResult(localRayResult, normalInWorldSpace);
					}
				}
			}
		} else {
			if (collisionShape.isConcave()) {
				if (collisionShape.getShapeType() == BroadphaseNativeType.TRIANGLE_MESH_SHAPE_PROXYTYPE) {
					// optimized version for BvhTriangleMeshShape
					BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape)collisionShape;
					Transform worldTocollisionObject = stack.allocTransform();
					worldTocollisionObject.inverse(colObjWorldTransform);
					Vector3 rayFromLocal = stack.alloc(rayFromTrans.origin);
					worldTocollisionObject.transform(rayFromLocal);
					Vector3 rayToLocal = stack.alloc(rayToTrans.origin);
					worldTocollisionObject.transform(rayToLocal);

					BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback,
						collisionObject, triangleMesh);
					rcb.hitFraction = resultCallback.closestHitFraction;
					triangleMesh.performRaycast(rcb, rayFromLocal, rayToLocal);
				} else {
					ConcaveShape triangleMesh = (ConcaveShape)collisionShape;

					Transform worldTocollisionObject = stack.allocTransform();
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3 rayFromLocal = stack.alloc(rayFromTrans.origin);
					worldTocollisionObject.transform(rayFromLocal);
					Vector3 rayToLocal = stack.alloc(rayToTrans.origin);
					worldTocollisionObject.transform(rayToLocal);

					BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback,
						collisionObject, triangleMesh);
					rcb.hitFraction = resultCallback.closestHitFraction;

					Vector3 rayAabbMinLocal = stack.alloc(rayFromLocal);
					VectorUtil.setMin(rayAabbMinLocal, rayToLocal);
					Vector3 rayAabbMaxLocal = stack.alloc(rayFromLocal);
					VectorUtil.setMax(rayAabbMaxLocal, rayToLocal);

					triangleMesh.processAllTriangles(rcb, rayAabbMinLocal, rayAabbMaxLocal);
				}
			} else {
				// todo: use AABB tree or other BVH acceleration structure!
				if (collisionShape.isCompound()) {
					CompoundShape compoundShape = (CompoundShape)collisionShape;
					int i = 0;
					Transform childTrans = stack.allocTransform();
					for (i = 0; i < compoundShape.getNumChildShapes(); i++) {
						compoundShape.getChildTransform(i, childTrans);
						CollisionShape childCollisionShape = compoundShape.getChildShape(i);
						Transform childWorldTrans = stack.alloc(colObjWorldTransform);
						childWorldTrans.mul(childTrans);
						// replace collision shape so that callback can determine the triangle
						CollisionShape saveCollisionShape = collisionObject.getCollisionShape();
						collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
						rayTestSingle(rayFromTrans, rayToTrans, collisionObject, childCollisionShape, childWorldTrans, resultCallback);
						// restore
						collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
					}
				}
			}
		}
		stack.leave();
	}

	private static class BridgeTriangleConvexcastCallback extends TriangleConvexcastCallback {
		public ConvexResultCallback resultCallback;
		public CollisionObject collisionObject;
		public TriangleMeshShape triangleMesh;
		public boolean normalInWorldSpace;

		public BridgeTriangleConvexcastCallback (ConvexShape castShape, Transform from, Transform to,
			ConvexResultCallback resultCallback, CollisionObject collisionObject, TriangleMeshShape triangleMesh,
			Transform triangleToWorld) {
			super(castShape, from, to, triangleToWorld, triangleMesh.getMargin());
			this.resultCallback = resultCallback;
			this.collisionObject = collisionObject;
			this.triangleMesh = triangleMesh;
		}

		@Override
		public float reportHit (Vector3 hitNormalLocal, Vector3 hitPointLocal, float hitFraction, int partId, int triangleIndex) {
			LocalShapeInfo shapeInfo = new LocalShapeInfo();
			shapeInfo.shapePart = partId;
			shapeInfo.triangleIndex = triangleIndex;
			if (hitFraction <= resultCallback.closestHitFraction) {
				LocalConvexResult convexResult = new LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal, hitPointLocal,
					hitFraction);
				return resultCallback.addSingleResult(convexResult, normalInWorldSpace);
			}
			return hitFraction;
		}
	}

	/** objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest. */
	public static void objectQuerySingle (ConvexShape castShape, Transform convexFromTrans, Transform convexToTrans,
		CollisionObject collisionObject, CollisionShape collisionShape, Transform colObjWorldTransform,
		ConvexResultCallback resultCallback, float allowedPenetration) {
		Stack stack = Stack.enter();
		if (collisionShape.isConvex()) {
			CastResult castResult = new CastResult();
			castResult.allowedPenetration = allowedPenetration;
			castResult.fraction = 1f; // ??

			ConvexShape convexShape = (ConvexShape)collisionShape;
			VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
			GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

			// JAVA TODO: should be convexCaster1
			// ContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
			GjkConvexCast convexCaster2 = new GjkConvexCast(castShape, convexShape, simplexSolver);
			// btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

			ConvexCast castPtr = convexCaster2;

			if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult)) {
				// add hit
				if (castResult.normal.len2() > 0.0001f) {
					if (castResult.fraction < resultCallback.closestHitFraction) {
						castResult.normal.nor();
						LocalConvexResult localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal,
							castResult.hitPoint, castResult.fraction);

						boolean normalInWorldSpace = true;
						resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
					}
				}
			}
		} else {
			if (collisionShape.isConcave()) {
				if (collisionShape.getShapeType() == BroadphaseNativeType.TRIANGLE_MESH_SHAPE_PROXYTYPE) {
					BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape)collisionShape;
					Transform worldTocollisionObject = stack.allocTransform();
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3 convexFromLocal = stack.allocVector3();
					convexFromLocal.set(convexFromTrans.origin);
					worldTocollisionObject.transform(convexFromLocal);

					Vector3 convexToLocal = stack.allocVector3();
					convexToLocal.set(convexToTrans.origin);
					worldTocollisionObject.transform(convexToLocal);

					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					Transform rotationXform = stack.allocTransform();
					Matrix3 tmpMat = stack.allocMatrix3();
					tmpMat.set(worldTocollisionObject.basis).mul(convexToTrans.basis);
					rotationXform.set(tmpMat);

					BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans,
						convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
					tccb.hitFraction = resultCallback.closestHitFraction;
					tccb.normalInWorldSpace = true;

					Vector3 boxMinLocal = stack.allocVector3();
					Vector3 boxMaxLocal = stack.allocVector3();
					castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
					triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
				} else {
					if (collisionShape.getShapeType() == BroadphaseNativeType.STATIC_PLANE_PROXYTYPE) {
						CastResult castResult = new CastResult();
						castResult.allowedPenetration = allowedPenetration;
						castResult.fraction = resultCallback.closestHitFraction;

						// btContinuousConvexCollision convexCaster1(castShape,planeShape);
						// btConvexCast* castPtr = &convexCaster1;

						/** Start buggy */
						StaticPlaneShape convexShape = (StaticPlaneShape)collisionShape;
						VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
						GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

						// JAVA TODO: should be convexCaster1
						// ContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
						GjkConvexCast convexCaster2 = null; // FIXME new GjkConvexCast(castShape, convexShape, simplexSolver);
						// btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

						ConvexCast castPtr = convexCaster2;
						/** end buggy */

						if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform,
							castResult)) {
							// add hit
							if (castResult.normal.len2() > 0.0001f) {
								if (castResult.fraction < resultCallback.closestHitFraction) {
									castResult.normal.nor();
									LocalConvexResult localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal,
										castResult.hitPoint, castResult.fraction);

									boolean normalInWorldSpace = true;
									resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
								}
							}
						}
						throw new UnsupportedOperationException(); // TODO implement
					} else {
						BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape)collisionShape;
						Transform worldTocollisionObject = stack.allocTransform();
						worldTocollisionObject.inverse(colObjWorldTransform);

						Vector3 convexFromLocal = stack.allocVector3();
						convexFromLocal.set(convexFromTrans.origin);
						worldTocollisionObject.transform(convexFromLocal);

						Vector3 convexToLocal = stack.allocVector3();
						convexToLocal.set(convexToTrans.origin);
						worldTocollisionObject.transform(convexToLocal);

						// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
						Transform rotationXform = stack.allocTransform();
						Matrix3 tmpMat = stack.allocMatrix3();
						tmpMat.set(worldTocollisionObject.basis).mul(convexToTrans.basis);
						rotationXform.set(tmpMat);

						BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans,
							convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
						tccb.hitFraction = resultCallback.closestHitFraction;
						tccb.normalInWorldSpace = false;
						Vector3 boxMinLocal = stack.allocVector3();
						Vector3 boxMaxLocal = stack.allocVector3();
						castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);

						Vector3 rayAabbMinLocal = stack.alloc(convexFromLocal);
						VectorUtil.setMin(rayAabbMinLocal, convexToLocal);
						Vector3 rayAabbMaxLocal = stack.alloc(convexFromLocal);
						VectorUtil.setMax(rayAabbMaxLocal, convexToLocal);
						rayAabbMinLocal.add(boxMinLocal);
						rayAabbMaxLocal.add(boxMaxLocal);
						triangleMesh.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal);
					}
				}
			} else {
				// todo: use AABB tree or other BVH acceleration structure!
				if (collisionShape.isCompound()) {
					CompoundShape compoundShape = (CompoundShape)collisionShape;
					for (int i = 0; i < compoundShape.getNumChildShapes(); i++) {
						Transform childTrans = compoundShape.getChildTransform(i, stack.allocTransform());
						CollisionShape childCollisionShape = compoundShape.getChildShape(i);
						Transform childWorldTrans = stack.allocTransform();
						childWorldTrans.mul(colObjWorldTransform, childTrans);
						// replace collision shape so that callback can determine the triangle
						CollisionShape saveCollisionShape = collisionObject.getCollisionShape();
						collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
						objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject, childCollisionShape,
							childWorldTrans, resultCallback, allowedPenetration);
						// restore
						collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
					}
				}
			}
		}
		stack.leave();
	}

	/** rayTest performs a raycast on all objects in the CollisionWorld, and calls the resultCallback. This allows for several
	 * queries: first hit, all hits, any hit, dependent on the value returned by the callback. */
	public void rayTest (Vector3 rayFromWorld, Vector3 rayToWorld, RayResultCallback resultCallback) {
		Stack stack = Stack.enter();
		Transform rayFromTrans = stack.allocTransform(), rayToTrans = stack.allocTransform();
		rayFromTrans.setIdentity();
		rayFromTrans.origin.set(rayFromWorld);
		rayToTrans.setIdentity();

		rayToTrans.origin.set(rayToWorld);

		// go over all objects, and if the ray intersects their aabb, do a ray-shape query using convexCaster (CCD)
		Vector3 collisionObjectAabbMin = stack.allocVector3(), collisionObjectAabbMax = stack.allocVector3();
		float[] hitLambda = new float[1];

		Transform tmpTrans = stack.allocTransform();

		for (int i = 0; i < collisionObjects.size(); i++) {
			// terminate further ray tests, once the closestHitFraction reached zero
			if (resultCallback.closestHitFraction == 0f) {
				break;
			}

			CollisionObject collisionObject = collisionObjects.getQuick(i);
			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
				// RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin,
					collisionObjectAabbMax);

				hitLambda[0] = resultCallback.closestHitFraction;
				Vector3 hitNormal = stack.allocVector3();
				if (AabbUtil2.rayAabb(rayFromWorld, rayToWorld, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda,
					hitNormal)) {
					rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionObject.getCollisionShape(),
						collisionObject.getWorldTransform(tmpTrans), resultCallback);
				}
			}

		}
		stack.leave();
	}

	/** convexTest performs a swept convex cast on all objects in the {@link CollisionWorld}, and calls the resultCallback This
	 * allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback. */
	public void convexSweepTest (ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld,
		ConvexResultCallback resultCallback) {
		Stack stack = Stack.enter();
		Transform convexFromTrans = stack.allocTransform();
		Transform convexToTrans = stack.allocTransform();

		convexFromTrans.set(convexFromWorld);
		convexToTrans.set(convexToWorld);

		Vector3 castShapeAabbMin = stack.allocVector3();
		Vector3 castShapeAabbMax = stack.allocVector3();

		// Compute AABB that encompasses angular movement
		{
			Vector3 linVel = stack.allocVector3();
			Vector3 angVel = stack.allocVector3();
			TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1f, linVel, angVel);
			Transform R = stack.allocTransform();
			R.setIdentity();
			R.setRotation(convexFromTrans.getRotation(stack.allocQuaternion()));
			castShape.calculateTemporalAabb(R, linVel, angVel, 1f, castShapeAabbMin, castShapeAabbMax);
		}

		Transform tmpTrans = stack.allocTransform();
		Vector3 collisionObjectAabbMin = stack.allocVector3();
		Vector3 collisionObjectAabbMax = stack.allocVector3();
		float[] hitLambda = new float[1];

		// go over all objects, and if the ray intersects their aabb + cast shape aabb,
		// do a ray-shape query using convexCaster (CCD)
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject collisionObject = collisionObjects.getQuick(i);

			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
				// RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				collisionObject.getWorldTransform(tmpTrans);
				collisionObject.getCollisionShape().getAabb(tmpTrans, collisionObjectAabbMin, collisionObjectAabbMax);
				AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
				hitLambda[0] = 1f; // could use resultCallback.closestHitFraction, but needs testing
				Vector3 hitNormal = stack.allocVector3();
				if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax,
					hitLambda, hitNormal)) {
					objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject, collisionObject.getCollisionShape(),
						tmpTrans, resultCallback, getDispatchInfo().allowedCcdPenetration);
				}
			}
		}
		stack.leave();
	}

	public ObjectArrayList<CollisionObject> getCollisionObjectArray () {
		return collisionObjects;
	}

	// //////////////////////////////////////////////////////////////////////////

	/** LocalShapeInfo gives extra information for complex shapes. Currently, only btTriangleMeshShape is available, so it just
	 * contains triangleIndex and subpart. */
	public static class LocalShapeInfo {
		public int shapePart;
		public int triangleIndex;
		// const btCollisionShape* m_shapeTemp;
		// const btTransform* m_shapeLocalTransform;
	}

	public static class LocalRayResult {
		public CollisionObject collisionObject;
		public LocalShapeInfo localShapeInfo;
		public final Vector3 hitNormalLocal = new Vector3();
		public float hitFraction;

		public LocalRayResult (CollisionObject collisionObject, LocalShapeInfo localShapeInfo, Vector3 hitNormalLocal,
			float hitFraction) {
			this.collisionObject = collisionObject;
			this.localShapeInfo = localShapeInfo;
			this.hitNormalLocal.set(hitNormalLocal);
			this.hitFraction = hitFraction;
		}
	}

	/** RayResultCallback is used to report new raycast results. */
	public static abstract class RayResultCallback {
		public float closestHitFraction = 1f;
		public CollisionObject collisionObject;
		public short collisionFilterGroup = CollisionFilterGroups.DEFAULT_FILTER;
		public short collisionFilterMask = CollisionFilterGroups.ALL_FILTER;

		public boolean hasHit () {
			return (collisionObject != null);
		}

		public boolean needsCollision (BroadphaseProxy proxy0) {
			boolean collides = ((proxy0.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
			collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
			return collides;
		}

		public abstract float addSingleResult (LocalRayResult rayResult, boolean normalInWorldSpace);
	}

	public static class ClosestRayResultCallback extends RayResultCallback {
		public final Vector3 rayFromWorld = new Vector3(); // used to calculate hitPointWorld from hitFraction
		public final Vector3 rayToWorld = new Vector3();

		public final Vector3 hitNormalWorld = new Vector3();
		public final Vector3 hitPointWorld = new Vector3();

		public ClosestRayResultCallback (Vector3 rayFromWorld, Vector3 rayToWorld) {
			this.rayFromWorld.set(rayFromWorld);
			this.rayToWorld.set(rayToWorld);
		}

		@Override
		public float addSingleResult (LocalRayResult rayResult, boolean normalInWorldSpace) {
			// caller already does the filter on the closestHitFraction
			assert (rayResult.hitFraction <= closestHitFraction);

			closestHitFraction = rayResult.hitFraction;
			collisionObject = rayResult.collisionObject;
			if (normalInWorldSpace) {
				hitNormalWorld.set(rayResult.hitNormalLocal);
			} else {
				// need to transform normal into worldspace
				hitNormalWorld.set(rayResult.hitNormalLocal);
				Stack stack = Stack.enter();
				hitNormalWorld.mul(collisionObject.getWorldTransform(stack.allocTransform()).basis);
				stack.leave();
			}

			VectorUtil.setInterpolate3(hitPointWorld, rayFromWorld, rayToWorld, rayResult.hitFraction);
			return rayResult.hitFraction;
		}
	}

	public static class LocalConvexResult {
		public CollisionObject hitCollisionObject;
		public LocalShapeInfo localShapeInfo;
		public final Vector3 hitNormalLocal = new Vector3();
		public final Vector3 hitPointLocal = new Vector3();
		public float hitFraction;

		public LocalConvexResult (CollisionObject hitCollisionObject, LocalShapeInfo localShapeInfo, Vector3 hitNormalLocal,
			Vector3 hitPointLocal, float hitFraction) {
			this.hitCollisionObject = hitCollisionObject;
			this.localShapeInfo = localShapeInfo;
			this.hitNormalLocal.set(hitNormalLocal);
			this.hitPointLocal.set(hitPointLocal);
			this.hitFraction = hitFraction;
		}
	}

	public static abstract class ConvexResultCallback {
		public float closestHitFraction = 1f;
		public short collisionFilterGroup = CollisionFilterGroups.DEFAULT_FILTER;
		public short collisionFilterMask = CollisionFilterGroups.ALL_FILTER;

		public boolean hasHit () {
			return (closestHitFraction < 1f);
		}

		public boolean needsCollision (BroadphaseProxy proxy0) {
			boolean collides = ((proxy0.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
			collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
			return collides;
		}

		public abstract float addSingleResult (LocalConvexResult convexResult, boolean normalInWorldSpace);
	}

	public static class ClosestConvexResultCallback extends ConvexResultCallback {
		public final Vector3 convexFromWorld = new Vector3(); // used to calculate hitPointWorld from hitFraction
		public final Vector3 convexToWorld = new Vector3();
		public final Vector3 hitNormalWorld = new Vector3();
		public final Vector3 hitPointWorld = new Vector3();
		public CollisionObject hitCollisionObject;

		public ClosestConvexResultCallback (Vector3 convexFromWorld, Vector3 convexToWorld) {
			this.convexFromWorld.set(convexFromWorld);
			this.convexToWorld.set(convexToWorld);
			this.hitCollisionObject = null;
		}

		@Override
		public float addSingleResult (LocalConvexResult convexResult, boolean normalInWorldSpace) {
			// caller already does the filter on the m_closestHitFraction
			assert (convexResult.hitFraction <= closestHitFraction);

			closestHitFraction = convexResult.hitFraction;
			hitCollisionObject = convexResult.hitCollisionObject;
			if (normalInWorldSpace) {
				hitNormalWorld.set(convexResult.hitNormalLocal);
				if (hitNormalWorld.len() > 2) {
					System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
				}
			} else {
				// need to transform normal into worldspace
				hitNormalWorld.set(convexResult.hitNormalLocal);
				Stack stack = Stack.enter();
				hitNormalWorld.mul(hitCollisionObject.getWorldTransform(stack.allocTransform()).basis);
				if (hitNormalWorld.len() > 2) {
					System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
				}
				stack.leave();
			}

			hitPointWorld.set(convexResult.hitPointLocal);
			return convexResult.hitFraction;
		}
	}

	private static class BridgeTriangleRaycastCallback extends TriangleRaycastCallback {
		public RayResultCallback resultCallback;
		public CollisionObject collisionObject;
		public ConcaveShape triangleMesh;

		public BridgeTriangleRaycastCallback (Vector3 from, Vector3 to, RayResultCallback resultCallback,
			CollisionObject collisionObject, ConcaveShape triangleMesh) {
			super(from, to);
			this.resultCallback = resultCallback;
			this.collisionObject = collisionObject;
			this.triangleMesh = triangleMesh;
		}

		public float reportHit (Vector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex) {
			LocalShapeInfo shapeInfo = new LocalShapeInfo();
			shapeInfo.shapePart = partId;
			shapeInfo.triangleIndex = triangleIndex;

			LocalRayResult rayResult = new LocalRayResult(collisionObject, shapeInfo, hitNormalLocal, hitFraction);

			boolean normalInWorldSpace = false;
			return resultCallback.addSingleResult(rayResult, normalInWorldSpace);
		}
	}

}
