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

package com.bulletphysics.dynamics;

import java.util.Comparator;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.broadphase.OverlappingPairCache;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.SimulationIslandManager;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;
import com.bulletphysics.linearmath.CProfileManager;
import com.bulletphysics.linearmath.Clock;
import com.bulletphysics.linearmath.DebugDrawModes;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;

/** DiscreteDynamicsWorld provides discrete rigid body simulation.
 * 
 * @author jezek2 */
public class DiscreteDynamicsWorld extends DynamicsWorld {

	protected ConstraintSolver constraintSolver;
	protected SimulationIslandManager islandManager;
	protected final ObjectArrayList<TypedConstraint> constraints = new ObjectArrayList<TypedConstraint>();
	protected final Vector3 gravity = new Vector3(0f, -10f, 0f);

	// for variable timesteps
	protected float localTime = 1f / 60f;
	// for variable timesteps

	protected boolean ownsIslandManager;
	protected boolean ownsConstraintSolver;

	protected ObjectArrayList<RaycastVehicle> vehicles = new ObjectArrayList<RaycastVehicle>();

	protected ObjectArrayList<ActionInterface> actions = new ObjectArrayList<ActionInterface>();

	protected int profileTimings = 0;

	public DiscreteDynamicsWorld (Dispatcher dispatcher, BroadphaseInterface pairCache, ConstraintSolver constraintSolver,
		CollisionConfiguration collisionConfiguration) {
		super(dispatcher, pairCache, collisionConfiguration);
		this.constraintSolver = constraintSolver;

		if (this.constraintSolver == null) {
			this.constraintSolver = new SequentialImpulseConstraintSolver();
			ownsConstraintSolver = true;
		} else {
			ownsConstraintSolver = false;
		}

		{
			islandManager = new SimulationIslandManager();
		}

		ownsIslandManager = true;
	}

	protected void saveKinematicState (float timeStep) {
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.getQuick(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				// Transform predictedTrans = new Transform();
				if (body.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
					if (body.isKinematicObject()) {
						// to calculate velocities next frame
						body.saveKinematicState(timeStep);
					}
				}
			}
		}
	}

	@Override
	public void debugDrawWorld () {
		Stack stack = Stack.enter();
		if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_CONTACT_POINTS) != 0) {
			int numManifolds = getDispatcher().getNumManifolds();
			Vector3 color = stack.allocVector3();
			color.set(0f, 0f, 0f);
			for (int i = 0; i < numManifolds; i++) {
				PersistentManifold contactManifold = getDispatcher().getManifoldByIndexInternal(i);
				// btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				// btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

				int numContacts = contactManifold.getNumContacts();
				for (int j = 0; j < numContacts; j++) {
					ManifoldPoint cp = contactManifold.getContactPoint(j);
					getDebugDrawer().drawContactPoint(cp.positionWorldOnB, cp.normalWorldOnB, cp.getDistance(), cp.getLifeTime(),
						color);
				}
			}
		}

		if (getDebugDrawer() != null
			&& (getDebugDrawer().getDebugMode() & (DebugDrawModes.DRAW_WIREFRAME | DebugDrawModes.DRAW_AABB)) != 0) {
			int i;

			Transform tmpTrans = stack.allocTransform();
			Vector3 minAabb = stack.allocVector3();
			Vector3 maxAabb = stack.allocVector3();
			Vector3 colorvec = stack.allocVector3();

			// todo: iterate over awake simulation islands!
			for (i = 0; i < collisionObjects.size(); i++) {
				CollisionObject colObj = collisionObjects.getQuick(i);
				if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
					Vector3 color = stack.allocVector3();
					color.set(255f, 255f, 255f);
					switch (colObj.getActivationState()) {
					case CollisionObject.ACTIVE_TAG:
						color.set(255f, 255f, 255f);
						break;
					case CollisionObject.ISLAND_SLEEPING:
						color.set(0f, 255f, 0f);
						break;
					case CollisionObject.WANTS_DEACTIVATION:
						color.set(0f, 255f, 255f);
						break;
					case CollisionObject.DISABLE_DEACTIVATION:
						color.set(255f, 0f, 0f);
						break;
					case CollisionObject.DISABLE_SIMULATION:
						color.set(255f, 255f, 0f);
						break;
					default: {
						color.set(255f, 0f, 0f);
					}
					}

					debugDrawObject(colObj.getWorldTransform(tmpTrans), colObj.getCollisionShape(), color);
				}
				if (debugDrawer != null && (debugDrawer.getDebugMode() & DebugDrawModes.DRAW_AABB) != 0) {
					colorvec.set(1f, 0f, 0f);
					colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
					debugDrawer.drawAabb(minAabb, maxAabb, colorvec);
				}
			}

			Vector3 wheelColor = stack.allocVector3();
			Vector3 wheelPosWS = stack.allocVector3();
			Vector3 axle = stack.allocVector3();
			Vector3 tmp = stack.allocVector3();

			for (i = 0; i < vehicles.size(); i++) {
				for (int v = 0; v < vehicles.getQuick(i).getNumWheels(); v++) {
					wheelColor.set(0, 255, 255);
					if (vehicles.getQuick(i).getWheelInfo(v).raycastInfo.isInContact) {
						wheelColor.set(0, 0, 255);
					} else {
						wheelColor.set(255, 0, 255);
					}

					wheelPosWS.set(vehicles.getQuick(i).getWheelInfo(v).worldTransform.origin);

					axle.set(MatrixUtil.getElement(vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis, 0, vehicles.getQuick(i)
						.getRightAxis()), MatrixUtil.getElement(vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis, 1, vehicles
						.getQuick(i).getRightAxis()), MatrixUtil.getElement(vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis,
						2, vehicles.getQuick(i).getRightAxis()));

					// m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
					// debug wheels (cylinders)
					tmp.set(wheelPosWS).add(axle);
					debugDrawer.drawLine(wheelPosWS, tmp, wheelColor);
					debugDrawer.drawLine(wheelPosWS, vehicles.getQuick(i).getWheelInfo(v).raycastInfo.contactPointWS, wheelColor);
				}
			}

			if (getDebugDrawer() != null && getDebugDrawer().getDebugMode() != 0) {
				for (i = 0; i < actions.size(); i++) {
					actions.getQuick(i).debugDraw(debugDrawer);
				}
			}
		}
		stack.leave();
	}

	@Override
	public void clearForces () {
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.getQuick(i);

			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				body.clearForces();
			}
		}
	}

	/** Apply gravity, call this once per timestep. */
	public void applyGravity () {
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.getQuick(i);

			RigidBody body = RigidBody.upcast(colObj);
			if (body != null && body.isActive()) {
				body.applyGravity();
			}
		}
	}

	protected void synchronizeMotionStates () {
		Stack stack = Stack.enter();
		Transform interpolatedTransform = stack.allocTransform();

		Transform tmpTrans = stack.allocTransform();
		Vector3 tmpLinVel = stack.allocVector3();
		Vector3 tmpAngVel = stack.allocVector3();

		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.getQuick(i);

			RigidBody body = RigidBody.upcast(colObj);
			if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
				// we need to call the update at least once, even for sleeping objects
				// otherwise the 'graphics' transform never updates properly
				// so todo: add 'dirty' flag
				// if (body->getActivationState() != ISLAND_SLEEPING)
				{
					TransformUtil.integrateTransform(body.getInterpolationWorldTransform(tmpTrans),
						body.getInterpolationLinearVelocity(tmpLinVel), body.getInterpolationAngularVelocity(tmpAngVel), localTime
							* body.getHitFraction(), interpolatedTransform);
					body.getMotionState().setWorldTransform(interpolatedTransform);
				}
			}
		}

		if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
			for (int i = 0; i < vehicles.size(); i++) {
				for (int v = 0; v < vehicles.getQuick(i).getNumWheels(); v++) {
					// synchronize the wheels with the (interpolated) chassis worldtransform
					vehicles.getQuick(i).updateWheelTransform(v, true);
				}
			}
		}
		stack.leave();
	}

	@Override
	public int stepSimulation (float timeStep, int maxSubSteps, float fixedTimeStep) {
		startProfiling(timeStep);

		long t0 = Clock.nanoTime();

		BulletStats.pushProfile("stepSimulation");
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		try {
			int numSimulationSubSteps = 0;

			if (maxSubSteps != 0) {
				// fixed timestep with interpolation
				localTime += timeStep;
				if (localTime >= fixedTimeStep) {
					numSimulationSubSteps = (int)(localTime / fixedTimeStep);
					localTime -= numSimulationSubSteps * fixedTimeStep;
				}
			} else {
				// variable timestep
				fixedTimeStep = timeStep;
				localTime = timeStep;
				if (ScalarUtil.fuzzyZero(timeStep)) {
					numSimulationSubSteps = 0;
					maxSubSteps = 0;
				} else {
					numSimulationSubSteps = 1;
					maxSubSteps = 1;
				}
			}

			// process some debugging flags
			if (getDebugDrawer() != null) {
				BulletGlobals.setDeactivationDisabled((getDebugDrawer().getDebugMode() & DebugDrawModes.NO_DEACTIVATION) != 0);
			}
			if (numSimulationSubSteps != 0) {
				saveKinematicState(fixedTimeStep);

				applyGravity();

				// clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
				int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

				for (int i = 0; i < clampedSimulationSteps; i++) {
					internalSingleStepSimulation(fixedTimeStep);
					synchronizeMotionStates();
				}
			}

			synchronizeMotionStates();

			clearForces();

			// #ifndef BT_NO_PROFILE
			CProfileManager.incrementFrameCounter();
			// #endif //BT_NO_PROFILE

			return numSimulationSubSteps;
		} finally {
			BulletStats.popProfile();

			BulletStats.stepSimulationTime = (Clock.nanoTime() - t0) / 1000000;
			stack.leave(sp);
		}
	}

	protected void internalSingleStepSimulation (float timeStep) {
		BulletStats.pushProfile("internalSingleStepSimulation");
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		try {
			// apply gravity, predict motion
			predictUnconstraintMotion(timeStep);

			DispatcherInfo dispatchInfo = getDispatchInfo();

			dispatchInfo.timeStep = timeStep;
			dispatchInfo.stepCount = 0;
			dispatchInfo.debugDraw = getDebugDrawer();

			// perform collision detection
			performDiscreteCollisionDetection();

			calculateSimulationIslands();

			getSolverInfo().timeStep = timeStep;

			// solve contact and other joint constraints
			solveConstraints(getSolverInfo());

			// CallbackTriggers();

			// integrate transforms
			integrateTransforms(timeStep);

			// update vehicle simulation
			updateActions(timeStep);

			// update vehicle simulation
			updateVehicles(timeStep);

			updateActivationState(timeStep);

			if (internalTickCallback != null) {
				internalTickCallback.internalTick(this, timeStep);
			}
		} finally {
			BulletStats.popProfile();
			stack.leave(sp);
		}
	}

	@Override
	public void setGravity (Vector3 gravity) {
		this.gravity.set(gravity);
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.getQuick(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				body.setGravity(gravity);
			}
		}
	}

	@Override
	public Vector3 getGravity (Vector3 out) {
		out.set(gravity);
		return out;
	}

	@Override
	public void removeRigidBody (RigidBody body) {
		removeCollisionObject(body);
	}

	@Override
	public void addRigidBody (RigidBody body) {
		if (!body.isStaticOrKinematicObject()) {
			body.setGravity(gravity);
		}

		if (body.getCollisionShape() != null) {
			boolean isDynamic = !(body.isStaticObject() || body.isKinematicObject());
			short collisionFilterGroup = isDynamic ? (short)CollisionFilterGroups.DEFAULT_FILTER
				: (short)CollisionFilterGroups.STATIC_FILTER;
			short collisionFilterMask = isDynamic ? (short)CollisionFilterGroups.ALL_FILTER
				: (short)(CollisionFilterGroups.ALL_FILTER ^ CollisionFilterGroups.STATIC_FILTER);

			addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
		}
	}

	public void addRigidBody (RigidBody body, short group, short mask) {
		if (!body.isStaticOrKinematicObject()) {
			body.setGravity(gravity);
		}

		if (body.getCollisionShape() != null) {
			addCollisionObject(body, group, mask);
		}
	}

	public void updateActions (float timeStep) {
		BulletStats.pushProfile("updateActions");
		try {
			for (int i = 0; i < actions.size(); i++) {
				actions.getQuick(i).updateAction(this, timeStep);
			}
		} finally {
			BulletStats.popProfile();
		}
	}

	protected void updateVehicles (float timeStep) {
		BulletStats.pushProfile("updateVehicles");
		try {
			for (int i = 0; i < vehicles.size(); i++) {
				RaycastVehicle vehicle = vehicles.getQuick(i);
				vehicle.updateVehicle(timeStep);
			}
		} finally {
			BulletStats.popProfile();
		}
	}

	protected void updateActivationState (float timeStep) {
		BulletStats.pushProfile("updateActivationState");
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		try {
			Vector3 tmp = stack.allocVector3();

			for (int i = 0; i < collisionObjects.size(); i++) {
				CollisionObject colObj = collisionObjects.getQuick(i);
				RigidBody body = RigidBody.upcast(colObj);
				if (body != null) {
					body.updateDeactivation(timeStep);

					if (body.wantsSleeping()) {
						if (body.isStaticOrKinematicObject()) {
							body.setActivationState(CollisionObject.ISLAND_SLEEPING);
						} else {
							if (body.getActivationState() == CollisionObject.ACTIVE_TAG) {
								body.setActivationState(CollisionObject.WANTS_DEACTIVATION);
							}
							if (body.getActivationState() == CollisionObject.ISLAND_SLEEPING) {
								tmp.set(0f, 0f, 0f);
								body.setAngularVelocity(tmp);
								body.setLinearVelocity(tmp);
							}
						}
					} else {
						if (body.getActivationState() != CollisionObject.DISABLE_DEACTIVATION) {
							body.setActivationState(CollisionObject.ACTIVE_TAG);
						}
					}
				}
			}
		} finally {
			stack.leave(sp);
			BulletStats.popProfile();
		}
	}

	@Override
	public void addConstraint (TypedConstraint constraint, boolean disableCollisionsBetweenLinkedBodies) {
		constraints.add(constraint);
		if (disableCollisionsBetweenLinkedBodies) {
			constraint.getRigidBodyA().addConstraintRef(constraint);
			constraint.getRigidBodyB().addConstraintRef(constraint);
		}
	}

	@Override
	public void removeConstraint (TypedConstraint constraint) {
		constraints.remove(constraint);
		constraint.getRigidBodyA().removeConstraintRef(constraint);
		constraint.getRigidBodyB().removeConstraintRef(constraint);
	}

	@Override
	public void addAction (ActionInterface action) {
		actions.add(action);
	}

	@Override
	public void removeAction (ActionInterface action) {
		actions.remove(action);
	}

	@Override
	public void addVehicle (RaycastVehicle vehicle) {
		vehicles.add(vehicle);
	}

	@Override
	public void removeVehicle (RaycastVehicle vehicle) {
		vehicles.remove(vehicle);
	}

	private static int getConstraintIslandId (TypedConstraint lhs) {
		int islandId;

		CollisionObject rcolObj0 = lhs.getRigidBodyA();
		CollisionObject rcolObj1 = lhs.getRigidBodyB();
		islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
		return islandId;
	}

	private static class InplaceSolverIslandCallback extends SimulationIslandManager.IslandCallback {
		public ContactSolverInfo solverInfo;
		public ConstraintSolver solver;
		public ObjectArrayList<TypedConstraint> sortedConstraints;
		public int numConstraints;
		public IDebugDraw debugDrawer;
		// public StackAlloc* m_stackAlloc;
		public Dispatcher dispatcher;

		public void init (ContactSolverInfo solverInfo, ConstraintSolver solver,
			ObjectArrayList<TypedConstraint> sortedConstraints, int numConstraints, IDebugDraw debugDrawer, Dispatcher dispatcher) {
			this.solverInfo = solverInfo;
			this.solver = solver;
			this.sortedConstraints = sortedConstraints;
			this.numConstraints = numConstraints;
			this.debugDrawer = debugDrawer;
			this.dispatcher = dispatcher;
		}

		public void processIsland (ObjectArrayList<CollisionObject> bodies, int numBodies,
			ObjectArrayList<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId) {
			if (islandId < 0) {
				// we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the
				// island id
				solver.solveGroup(bodies, numBodies, manifolds, manifolds_offset, numManifolds, sortedConstraints, 0, numConstraints,
					solverInfo, debugDrawer/* ,m_stackAlloc */, dispatcher);
			} else {
				// also add all non-contact constraints/joints for this island
				// ObjectArrayList<TypedConstraint> startConstraint = null;
				int startConstraint_idx = -1;
				int numCurConstraints = 0;
				int i;

				// find the first constraint for this island
				for (i = 0; i < numConstraints; i++) {
					if (getConstraintIslandId(sortedConstraints.getQuick(i)) == islandId) {
						// startConstraint = &m_sortedConstraints[i];
						// startConstraint = sortedConstraints.subList(i, sortedConstraints.size());
						startConstraint_idx = i;
						break;
					}
				}
				// count the number of constraints in this island
				for (; i < numConstraints; i++) {
					if (getConstraintIslandId(sortedConstraints.getQuick(i)) == islandId) {
						numCurConstraints++;
					}
				}

				// only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
				if ((numManifolds + numCurConstraints) > 0) {
					solver.solveGroup(bodies, numBodies, manifolds, manifolds_offset, numManifolds, sortedConstraints,
						startConstraint_idx, numCurConstraints, solverInfo, debugDrawer/* ,m_stackAlloc */, dispatcher);
				}
			}
		}
	}

	private ObjectArrayList<TypedConstraint> sortedConstraints = new ObjectArrayList<TypedConstraint>();
	private InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback();

	protected void solveConstraints (ContactSolverInfo solverInfo) {
		BulletStats.pushProfile("solveConstraints");
		try {
			// sorted version of all btTypedConstraint, based on islandId
			sortedConstraints.clear();
			for (int i = 0; i < constraints.size(); i++) {
				sortedConstraints.add(constraints.getQuick(i));
			}
			// Collections.sort(sortedConstraints, sortConstraintOnIslandPredicate);
			MiscUtil.quickSort(sortedConstraints, sortConstraintOnIslandPredicate);

			ObjectArrayList<TypedConstraint> constraintsPtr = getNumConstraints() != 0 ? sortedConstraints : null;

			solverCallback.init(solverInfo, constraintSolver, constraintsPtr, sortedConstraints.size(),
				debugDrawer/* ,m_stackAlloc */, dispatcher1);

			constraintSolver.prepareSolve(getCollisionWorld().getNumCollisionObjects(), getCollisionWorld().getDispatcher()
				.getNumManifolds());

			// solve all the constraints for this island
			islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(), getCollisionWorld().getCollisionObjectArray(),
				solverCallback);

			constraintSolver.allSolved(solverInfo, debugDrawer/* , m_stackAlloc */);
		} finally {
			BulletStats.popProfile();
		}
	}

	protected void calculateSimulationIslands () {
		BulletStats.pushProfile("calculateSimulationIslands");
		try {
			getSimulationIslandManager().updateActivationState(getCollisionWorld(), getCollisionWorld().getDispatcher());

			{
				int i;
				int numConstraints = constraints.size();
				for (i = 0; i < numConstraints; i++) {
					TypedConstraint constraint = constraints.getQuick(i);

					RigidBody colObj0 = constraint.getRigidBodyA();
					RigidBody colObj1 = constraint.getRigidBodyB();

					if (((colObj0 != null) && (!colObj0.isStaticOrKinematicObject()))
						&& ((colObj1 != null) && (!colObj1.isStaticOrKinematicObject()))) {
						if (colObj0.isActive() || colObj1.isActive()) {
							getSimulationIslandManager().getUnionFind().unite((colObj0).getIslandTag(), (colObj1).getIslandTag());
						}
					}
				}
			}

			// Store the island id in each body
			getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());
		} finally {
			BulletStats.popProfile();
		}
	}

	protected void integrateTransforms (float timeStep) {
		BulletStats.pushProfile("integrateTransforms");
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		try {
			Vector3 tmp = stack.allocVector3();
			Transform tmpTrans = stack.allocTransform();

			Transform predictedTrans = stack.allocTransform();
			for (int i = 0; i < collisionObjects.size(); i++) {
				CollisionObject colObj = collisionObjects.getQuick(i);
				RigidBody body = RigidBody.upcast(colObj);
				if (body != null) {
					body.setHitFraction(1f);

					if (body.isActive() && (!body.isStaticOrKinematicObject())) {
						body.predictIntegratedTransform(timeStep, predictedTrans);

						tmp.set(predictedTrans.origin).sub(body.getWorldTransform(tmpTrans).origin);
						float squareMotion = tmp.len2();

						if (body.getCcdSquareMotionThreshold() != 0f && body.getCcdSquareMotionThreshold() < squareMotion) {
							BulletStats.pushProfile("CCD motion clamping");
							try {
								if (body.getCollisionShape().isConvex()) {
									BulletStats.gNumClampedCcdMotions++;

									ClosestNotMeConvexResultCallback sweepResults = new ClosestNotMeConvexResultCallback(body,
										body.getWorldTransform(tmpTrans).origin, predictedTrans.origin, getBroadphase()
											.getOverlappingPairCache(), getDispatcher());
									// ConvexShape convexShape = (ConvexShape)body.getCollisionShape();
									SphereShape tmpSphere = new SphereShape(body.getCcdSweptSphereRadius()); // btConvexShape* convexShape
																																		// =
																																		// static_cast<btConvexShape*>(body->getCollisionShape());

									sweepResults.collisionFilterGroup = body.getBroadphaseProxy().collisionFilterGroup;
									sweepResults.collisionFilterMask = body.getBroadphaseProxy().collisionFilterMask;

									convexSweepTest(tmpSphere, body.getWorldTransform(tmpTrans), predictedTrans, sweepResults);
									// JAVA NOTE: added closestHitFraction test to prevent objects being stuck
									if (sweepResults.hasHit() && (sweepResults.closestHitFraction > 0.0001f)) {
										body.setHitFraction(sweepResults.closestHitFraction);
										body.predictIntegratedTransform(timeStep * body.getHitFraction(), predictedTrans);
										body.setHitFraction(0f);
										// System.out.printf("clamped integration to hit fraction = %f\n", sweepResults.closestHitFraction);
									}
								}
							} finally {
								BulletStats.popProfile();
							}
						}

						body.proceedToTransform(predictedTrans);
					}
				}
			}
		} finally {
			stack.leave(sp);
			BulletStats.popProfile();
		}
	}

	protected void predictUnconstraintMotion (float timeStep) {
		BulletStats.pushProfile("predictUnconstraintMotion");
		Stack stack = Stack.enter();
		int sp = stack.getSp();
		try {
			Transform tmpTrans = stack.allocTransform();

			for (int i = 0; i < collisionObjects.size(); i++) {
				CollisionObject colObj = collisionObjects.getQuick(i);
				RigidBody body = RigidBody.upcast(colObj);
				if (body != null) {
					if (!body.isStaticOrKinematicObject()) {
						if (body.isActive()) {
							body.integrateVelocities(timeStep);
							// damping
							body.applyDamping(timeStep);

							body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
						}
					}
				}
			}
		} finally {
			stack.leave(sp);
			BulletStats.popProfile();
		}
	}

	protected void startProfiling (float timeStep) {
		// #ifndef BT_NO_PROFILE
		CProfileManager.reset();
		// #endif //BT_NO_PROFILE
	}

	protected void debugDrawSphere (float radius, Transform transform, Vector3 color) {
		Stack stack = Stack.enter();
		Vector3 start = stack.alloc(transform.origin);

		Vector3 xoffs = stack.allocVector3();
		xoffs.set(radius, 0, 0);
		xoffs.mul(transform.basis);
		Vector3 yoffs = stack.allocVector3();
		yoffs.set(0, radius, 0);
		yoffs.mul(transform.basis);
		Vector3 zoffs = stack.allocVector3();
		zoffs.set(0, 0, radius);
		zoffs.mul(transform.basis);

		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		// XY
		tmp1.set(start).sub(xoffs);
		tmp2.set(start).add(yoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(yoffs);
		tmp2.set(start).add(xoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(xoffs);
		tmp2.set(start).sub(yoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).sub(yoffs);
		tmp2.set(start).sub(xoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);

		// XZ
		tmp1.set(start).sub(xoffs);
		tmp2.set(start).add(zoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(zoffs);
		tmp2.set(start).add(xoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(xoffs);
		tmp2.set(start).sub(zoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).sub(zoffs);
		tmp2.set(start).sub(xoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);

		// YZ
		tmp1.set(start).sub(yoffs);
		tmp2.set(start).add(zoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(zoffs);
		tmp2.set(start).add(yoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).add(yoffs);
		tmp2.set(start).sub(zoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		tmp1.set(start).sub(zoffs);
		tmp2.set(start).sub(yoffs);
		getDebugDrawer().drawLine(tmp1, tmp2, color);
		stack.leave();
	}

	public void debugDrawObject (Transform worldTransform, CollisionShape shape, Vector3 color) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		// Draw a small simplex at the center of the object
		{
			Vector3 start = stack.alloc(worldTransform.origin);

			tmp.set(1f, 0f, 0f);
			tmp.mul(worldTransform.basis);
			tmp.add(start);
			tmp2.set(1f, 0f, 0f);
			getDebugDrawer().drawLine(start, tmp, tmp2);

			tmp.set(0f, 1f, 0f);
			tmp.mul(worldTransform.basis);
			tmp.add(start);
			tmp2.set(0f, 1f, 0f);
			getDebugDrawer().drawLine(start, tmp, tmp2);

			tmp.set(0f, 0f, 1f);
			tmp.mul(worldTransform.basis);
			tmp.add(start);
			tmp2.set(0f, 0f, 1f);
			getDebugDrawer().drawLine(start, tmp, tmp2);
		}

		// JAVA TODO: debugDrawObject, note that this commented code is from old version, use actual version when implementing

// if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
// {
// const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
// for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
// {
// btTransform childTrans = compoundShape->getChildTransform(i);
// const btCollisionShape* colShape = compoundShape->getChildShape(i);
// debugDrawObject(worldTransform*childTrans,colShape,color);
// }
//
// } else
// {
// switch (shape->getShapeType())
// {
//
// case SPHERE_SHAPE_PROXYTYPE:
// {
// const btSphereShape* sphereShape = static_cast<const btSphereShape*>(shape);
// btScalar radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
//
// debugDrawSphere(radius, worldTransform, color);
// break;
// }
// case MULTI_SPHERE_SHAPE_PROXYTYPE:
// {
// const btMultiSphereShape* multiSphereShape = static_cast<const btMultiSphereShape*>(shape);
//
// for (int i = multiSphereShape->getSphereCount()-1; i>=0;i--)
// {
// btTransform childTransform = worldTransform;
// childTransform.getOrigin() += multiSphereShape->getSpherePosition(i);
// debugDrawSphere(multiSphereShape->getSphereRadius(i), childTransform, color);
// }
//
// break;
// }
// case CAPSULE_SHAPE_PROXYTYPE:
// {
// const btCapsuleShape* capsuleShape = static_cast<const btCapsuleShape*>(shape);
//
// btScalar radius = capsuleShape->getRadius();
// btScalar halfHeight = capsuleShape->getHalfHeight();
//
// // Draw the ends
// {
// btTransform childTransform = worldTransform;
// childTransform.getOrigin() = worldTransform * btVector3(0,halfHeight,0);
// debugDrawSphere(radius, childTransform, color);
// }
//
// {
// btTransform childTransform = worldTransform;
// childTransform.getOrigin() = worldTransform * btVector3(0,-halfHeight,0);
// debugDrawSphere(radius, childTransform, color);
// }
//
// // Draw some additional lines
// btVector3 start = worldTransform.getOrigin();
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * btVector3(-radius,halfHeight,0),start+worldTransform.getBasis() *
// btVector3(-radius,-halfHeight,0), color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * btVector3(radius,halfHeight,0),start+worldTransform.getBasis() *
// btVector3(radius,-halfHeight,0), color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * btVector3(0,halfHeight,-radius),start+worldTransform.getBasis() *
// btVector3(0,-halfHeight,-radius), color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * btVector3(0,halfHeight,radius),start+worldTransform.getBasis() *
// btVector3(0,-halfHeight,radius), color);
//
// break;
// }
// case CONE_SHAPE_PROXYTYPE:
// {
// const btConeShape* coneShape = static_cast<const btConeShape*>(shape);
// btScalar radius = coneShape->getRadius();//+coneShape->getMargin();
// btScalar height = coneShape->getHeight();//+coneShape->getMargin();
// btVector3 start = worldTransform.getOrigin();
//
// int upAxis= coneShape->getConeUpIndex();
//
//
// btVector3 offsetHeight(0,0,0);
// offsetHeight[upAxis] = height * btScalar(0.5);
// btVector3 offsetRadius(0,0,0);
// offsetRadius[(upAxis+1)%3] = radius;
// btVector3 offset2Radius(0,0,0);
// offset2Radius[(upAxis+2)%3] = radius;
//
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() *
// (-offsetHeight+offsetRadius),color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() *
// (-offsetHeight-offsetRadius),color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() *
// (-offsetHeight+offset2Radius),color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight),start+worldTransform.getBasis() *
// (-offsetHeight-offset2Radius),color);
//
//
//
// break;
//
// }
// case CYLINDER_SHAPE_PROXYTYPE:
// {
// const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
// int upAxis = cylinder->getUpAxis();
// btScalar radius = cylinder->getRadius();
// btScalar halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];
// btVector3 start = worldTransform.getOrigin();
// btVector3 offsetHeight(0,0,0);
// offsetHeight[upAxis] = halfHeight;
// btVector3 offsetRadius(0,0,0);
// offsetRadius[(upAxis+1)%3] = radius;
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight+offsetRadius),start+worldTransform.getBasis() *
// (-offsetHeight+offsetRadius),color);
// getDebugDrawer()->drawLine(start+worldTransform.getBasis() * (offsetHeight-offsetRadius),start+worldTransform.getBasis() *
// (-offsetHeight-offsetRadius),color);
// break;
// }
// default:
// {
//
// if (shape->isConcave())
// {
// btConcaveShape* concaveMesh = (btConcaveShape*) shape;
//
// //todo pass camera, for some culling
// btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
// btVector3 aabbMin(btScalar(-1e30),btScalar(-1e30),btScalar(-1e30));
//
// DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
// concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);
//
// }
//
// if (shape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
// {
// btConvexTriangleMeshShape* convexMesh = (btConvexTriangleMeshShape*) shape;
// //todo: pass camera for some culling
// btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
// btVector3 aabbMin(btScalar(-1e30),btScalar(-1e30),btScalar(-1e30));
// //DebugDrawcallback drawCallback;
// DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
// convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);
// }
//
//
// /// for polyhedral shapes
// if (shape->isPolyhedral())
// {
// btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;
//
// int i;
// for (i=0;i<polyshape->getNumEdges();i++)
// {
// btPoint3 a,b;
// polyshape->getEdge(i,a,b);
// btVector3 wa = worldTransform * a;
// btVector3 wb = worldTransform * b;
// getDebugDrawer()->drawLine(wa,wb,color);
//
// }
//
//
// }
// }
// }
// }
		stack.leave();
	}

	@Override
	public void setConstraintSolver (ConstraintSolver solver) {
		if (ownsConstraintSolver) {
			// btAlignedFree( m_constraintSolver);
		}
		ownsConstraintSolver = false;
		constraintSolver = solver;
	}

	@Override
	public ConstraintSolver getConstraintSolver () {
		return constraintSolver;
	}

	@Override
	public int getNumConstraints () {
		return constraints.size();
	}

	@Override
	public TypedConstraint getConstraint (int index) {
		return constraints.getQuick(index);
	}

	// JAVA NOTE: not part of the original api
	@Override
	public int getNumActions () {
		return actions.size();
	}

	// JAVA NOTE: not part of the original api
	@Override
	public ActionInterface getAction (int index) {
		return actions.getQuick(index);
	}

	public SimulationIslandManager getSimulationIslandManager () {
		return islandManager;
	}

	public CollisionWorld getCollisionWorld () {
		return this;
	}

	@Override
	public DynamicsWorldType getWorldType () {
		return DynamicsWorldType.DISCRETE_DYNAMICS_WORLD;
	}

	public void setNumTasks (int numTasks) {
	}

	// //////////////////////////////////////////////////////////////////////////

	private static final Comparator<TypedConstraint> sortConstraintOnIslandPredicate = new Comparator<TypedConstraint>() {
		public int compare (TypedConstraint lhs, TypedConstraint rhs) {
			int rIslandId0, lIslandId0;
			rIslandId0 = getConstraintIslandId(rhs);
			lIslandId0 = getConstraintIslandId(lhs);
			return lIslandId0 < rIslandId0 ? -1 : +1;
		}
	};

// private static class DebugDrawcallback implements TriangleCallback, InternalTriangleIndexCallback {
// private IDebugDraw debugDrawer;
// private final Vector3 color = new Vector3();
// private final Transform worldTrans = new Transform();
//
// public DebugDrawcallback(IDebugDraw debugDrawer, Transform worldTrans, Vector3 color) {
// this.debugDrawer = debugDrawer;
// this.worldTrans.set(worldTrans);
// this.color.set(color);
// }
//
// public void internalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex) {
// processTriangle(triangle,partId,triangleIndex);
// }
//
// private final Vector3 wv0 = new Vector3(),wv1 = new Vector3(),wv2 = new Vector3();
//
// public void processTriangle(Vector3[] triangle, int partId, int triangleIndex) {
// wv0.set(triangle[0]);
// worldTrans.transform(wv0);
// wv1.set(triangle[1]);
// worldTrans.transform(wv1);
// wv2.set(triangle[2]);
// worldTrans.transform(wv2);
//
// debugDrawer.drawLine(wv0, wv1, color);
// debugDrawer.drawLine(wv1, wv2, color);
// debugDrawer.drawLine(wv2, wv0, color);
// }
// }

	private static class ClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {
		private CollisionObject me;
		private float allowedPenetration = 0f;
		private OverlappingPairCache pairCache;
		private Dispatcher dispatcher;

		public ClosestNotMeConvexResultCallback (CollisionObject me, Vector3 fromA, Vector3 toA, OverlappingPairCache pairCache,
			Dispatcher dispatcher) {
			super(fromA, toA);
			this.me = me;
			this.pairCache = pairCache;
			this.dispatcher = dispatcher;
		}

		@Override
		public float addSingleResult (LocalConvexResult convexResult, boolean normalInWorldSpace) {
			if (convexResult.hitCollisionObject == me) {
				return 1f;
			}

			Stack stack = Stack.enter();
			Vector3 linVelA = stack.allocVector3(), linVelB = stack.allocVector3();
			linVelA.set(convexToWorld).sub(convexFromWorld);
			linVelB.set(0f, 0f, 0f);// toB.getOrigin()-fromB.getOrigin();

			Vector3 relativeVelocity = stack.allocVector3();
			relativeVelocity.set(linVelA).sub(linVelB);
			// don't report time of impact for motion away from the contact normal (or causes minor penetration)
			if (convexResult.hitNormalLocal.dot(relativeVelocity) >= -allowedPenetration) {
				stack.leave();
				return 1f;
			}

			stack.leave();
			return super.addSingleResult(convexResult, normalInWorldSpace);
		}

		@Override
		public boolean needsCollision (BroadphaseProxy proxy0) {
			// don't collide with itself
			if (proxy0.clientObject == me) {
				return false;
			}

			// don't do CCD when the collision filters are not matching
			if (!super.needsCollision(proxy0)) {
				return false;
			}

			CollisionObject otherObj = (CollisionObject)proxy0.clientObject;

			// call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
			if (dispatcher.needsResponse(me, otherObj)) {
				// don't do CCD when there are already contact points (touching contact/penetration)
				ObjectArrayList<PersistentManifold> manifoldArray = new ObjectArrayList<PersistentManifold>();
				BroadphasePair collisionPair = pairCache.findPair(me.getBroadphaseHandle(), proxy0);
				if (collisionPair != null) {
					if (collisionPair.algorithm != null) {
						// manifoldArray.resize(0);
						collisionPair.algorithm.getAllContactManifolds(manifoldArray);
						for (int j = 0; j < manifoldArray.size(); j++) {
							PersistentManifold manifold = manifoldArray.getQuick(j);
							if (manifold.getNumContacts() > 0) {
								return false;
							}
						}
					}
				}
			}
			return true;
		}
	}

}
