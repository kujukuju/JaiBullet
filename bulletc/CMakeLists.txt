cmake_minimum_required(VERSION 3.8)

# https://cmake.org/cmake/help/latest/command/project.html
project(bullet3 VERSION 2.4.1)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

option(BUILD_SHARED_LIBS "Build bullet as a shared library" ON)
option(BUILD_SIMD "Build bullet using simd" ON)

ADD_DEFINITIONS(-DB3_USE_CLEW)

IF (NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "Debug")
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

OPTION(USE_DOUBLE_PRECISION "Use double precision"	OFF)
SET(CLAMP_VELOCITIES "0" CACHE STRING "Clamp rigid bodies' velocity to this value, if larger than zero. Useful to prevent floating point errors or in general runaway velocities in complex scenarios")
OPTION(BULLET2_MULTITHREADING "Build Bullet 2 libraries with mutex locking around certain operations (required for multi-threading)" OFF)

IF (BUILD_SHARED_LIBS)
    ADD_DEFINITIONS( -DBT_SHARED)
ENDIF (BUILD_SHARED_LIBS)

# I'm not sure what will happen when I try to compile for wasm because it seems to check if your platform
# IF (BUILD_SIMD)
#     ADD_DEFINITIONS( -DBT_USE_SSE)
# ENDIF (BUILD_SIMD)

IF (CLAMP_VELOCITIES)
    ADD_DEFINITIONS( -DBT_CLAMP_VELOCITY_TO=${CLAMP_VELOCITIES})
ENDIF (CLAMP_VELOCITIES)

IF (USE_DOUBLE_PRECISION)
    ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
    SET( BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")
ENDIF (USE_DOUBLE_PRECISION)

IF (BULLET2_MULTITHREADING)
    OPTION(BULLET2_USE_OPEN_MP_MULTITHREADING "Build Bullet 2 with support for multi-threading with OpenMP (requires a compiler with OpenMP support)" OFF)
    OPTION(BULLET2_USE_TBB_MULTITHREADING "Build Bullet 2 with support for multi-threading with Intel Threading Building Blocks (requires the TBB library to be already installed)" OFF)
    IF (MSVC)
        OPTION(BULLET2_USE_PPL_MULTITHREADING "Build Bullet 2 with support for multi-threading with Microsoft Parallel Patterns Library (requires MSVC compiler)" OFF)
    ENDIF (MSVC)
ENDIF (BULLET2_MULTITHREADING)

IF(BULLET2_MULTITHREADING)
    ADD_DEFINITIONS( -DBT_THREADSAFE=1 )
    IF (NOT MSVC)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    ENDIF (NOT MSVC)
    IF (NOT WIN32)
        FIND_PACKAGE(Threads)
        LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} )
    ENDIF (NOT WIN32)
ENDIF (BULLET2_MULTITHREADING)

include_directories(bullet3-3.24)
include_directories(capi)

set(BULLET3_SOURCE_FILES
        # ./btBulletCollisionAll.cpp
        # ./btBulletDynamicsAll.cpp
        # ./btLinearMathAll.cpp
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3DynamicBvh.cpp
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.cpp
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.cpp
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.cpp
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.cpp
        ./bullet3-3.24/Bullet3Common/b3AlignedAllocator.cpp
        ./bullet3-3.24/Bullet3Common/b3Logging.cpp
        ./bullet3-3.24/Bullet3Common/b3Vector3.cpp
        ./bullet3-3.24/Bullet3Dynamics/b3CpuRigidBodyPipeline.cpp
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.cpp
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.cpp
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.cpp
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.cpp
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.cpp
        ./bullet3-3.24/Bullet3Geometry/b3ConvexHullComputer.cpp
        ./bullet3-3.24/Bullet3Geometry/b3GeometryUtil.cpp
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuGridBroadphase.cpp
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvh.cpp
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.cpp
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.cpp
        ./bullet3-3.24/Bullet3OpenCL/Initialize/b3OpenCLUtils.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3ConvexHullContact.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3GjkEpa.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3OptimizedBvh.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3QuantizedBvh.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3StridingMeshInterface.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3TriangleCallback.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3TriangleIndexVertexArray.cpp
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3VoronoiSimplexSolver.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3BoundSearchCL.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3FillCL.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3PrefixScanCL.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3PrefixScanFloat4CL.cpp
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.cpp
        ./bullet3-3.24/Bullet3OpenCL/Raycast/b3GpuRaycast.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuGenericConstraint.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuJacobiContactSolver.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuPgsConstraintSolver.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuPgsContactSolver.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.cpp
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3Solver.cpp
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3BulletFile.cpp
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Chunk.cpp
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3DNA.cpp
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3File.cpp
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Serializer.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvt.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDispatcher.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionObject.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btGhostObject.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btManifoldResult.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btUnionFind.cpp
        ./bullet3-3.24/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBox2dShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBoxShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCapsuleShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCompoundShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConcaveShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConeShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvex2dShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexHullShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCylinderShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btEmptyShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMiniSDF.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMultiSphereShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btSdfCollisionShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btShapeHull.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTetrahedronShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleBuffer.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleCallback.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
        ./bullet3-3.24/BulletCollision/CollisionShapes/btUniformScalingShape.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btContactProcessing.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btGenericPoolAllocator.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactBvh.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactShape.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/btTriangleShapeEx.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/gim_box_set.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/gim_contact.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/gim_memory.cpp
        ./bullet3-3.24/BulletCollision/Gimpact/gim_tri_collision.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
        ./bullet3-3.24/BulletDynamics/Character/btKinematicCharacterController.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btBatchedConstraints.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGearConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp
        ./bullet3-3.24/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp
        ./bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.cpp
        ./bullet3-3.24/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp
        ./bullet3-3.24/BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBody.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyGearConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.cpp
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.cpp
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp
        ./bullet3-3.24/BulletDynamics/Vehicle/btRaycastVehicle.cpp
        ./bullet3-3.24/BulletDynamics/Vehicle/btWheelInfo.cpp
        ./bullet3-3.24/BulletInverseDynamics/details/MultiBodyTreeImpl.cpp
        ./bullet3-3.24/BulletInverseDynamics/details/MultiBodyTreeInitCache.cpp
        ./bullet3-3.24/BulletInverseDynamics/IDMath.cpp
        ./bullet3-3.24/BulletInverseDynamics/MultiBodyTree.cpp
        ./bullet3-3.24/BulletSoftBody/btDefaultSoftBodySolver.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableBackwardEulerObjective.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableBodySolver.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableContactConstraint.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableContactProjection.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableMultiBodyConstraintSolver.cpp
        ./bullet3-3.24/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftBody.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftBodyHelpers.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftMultiBodyDynamicsWorld.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftRigidDynamicsWorld.cpp
        ./bullet3-3.24/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.cpp
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.cpp
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.cpp
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableContactConstraint.cpp
        ./bullet3-3.24/BulletSoftBody/poly34.cpp
        ./bullet3-3.24/cmake-build-debug/CMakeFiles/3.17.5/CompilerIdCXX/CMakeCXXCompilerId.cpp
        ./bullet3-3.24/LinearMath/btAlignedAllocator.cpp
        ./bullet3-3.24/LinearMath/btConvexHull.cpp
        ./bullet3-3.24/LinearMath/btConvexHullComputer.cpp
        ./bullet3-3.24/LinearMath/btGeometryUtil.cpp
        ./bullet3-3.24/LinearMath/btPolarDecomposition.cpp
        ./bullet3-3.24/LinearMath/btQuickprof.cpp
        ./bullet3-3.24/LinearMath/btReducedVector.cpp
        ./bullet3-3.24/LinearMath/btSerializer.cpp
        ./bullet3-3.24/LinearMath/btSerializer64.cpp
        ./bullet3-3.24/LinearMath/btThreads.cpp
        ./bullet3-3.24/LinearMath/btVector3.cpp
        ./bullet3-3.24/LinearMath/TaskScheduler/btTaskScheduler.cpp
        ./bullet3-3.24/LinearMath/TaskScheduler/btThreadSupportPosix.cpp
        ./bullet3-3.24/LinearMath/TaskScheduler/btThreadSupportWin32.cpp
        ./bullet3-3.24/clew/clew.c
        ./capi/capi.cpp)

set(BULLET3_HEADER_FILES
        # ./btBulletCollisionCommon.h
        # ./btBulletDynamicsCommon.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3BroadphaseCallback.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3DynamicBvh.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3OverlappingPair.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.h
        ./bullet3-3.24/Bullet3Collision/BroadPhaseCollision/shared/b3Aabb.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3Config.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3Contact4.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3BvhSubtreeInfoData.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3BvhTraversal.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3ClipFaces.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3Contact4Data.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3ContactConvexConvexSAT.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3ContactSphereSphere.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3ConvexPolyhedronData.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3FindConcaveSatAxis.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3FindSeparatingAxis.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3MprPenetration.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3NewContactReduction.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3QuantizedBvhNodeData.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3ReduceContacts.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h
        ./bullet3-3.24/Bullet3Collision/NarrowPhaseCollision/shared/b3UpdateAabbs.h
        ./bullet3-3.24/Bullet3Common/b3AlignedAllocator.h
        ./bullet3-3.24/Bullet3Common/b3AlignedObjectArray.h
        ./bullet3-3.24/Bullet3Common/b3CommandLineArgs.h
        ./bullet3-3.24/Bullet3Common/b3FileUtils.h
        ./bullet3-3.24/Bullet3Common/b3HashMap.h
        ./bullet3-3.24/Bullet3Common/b3Logging.h
        ./bullet3-3.24/Bullet3Common/b3Matrix3x3.h
        ./bullet3-3.24/Bullet3Common/b3MinMax.h
        ./bullet3-3.24/Bullet3Common/b3PoolAllocator.h
        ./bullet3-3.24/Bullet3Common/b3QuadWord.h
        ./bullet3-3.24/Bullet3Common/b3Quaternion.h
        ./bullet3-3.24/Bullet3Common/b3Random.h
        ./bullet3-3.24/Bullet3Common/b3ResizablePool.h
        ./bullet3-3.24/Bullet3Common/b3Scalar.h
        ./bullet3-3.24/Bullet3Common/b3StackAlloc.h
        ./bullet3-3.24/Bullet3Common/b3Transform.h
        ./bullet3-3.24/Bullet3Common/b3TransformUtil.h
        ./bullet3-3.24/Bullet3Common/b3Vector3.h
        ./bullet3-3.24/Bullet3Common/shared/b3Float4.h
        ./bullet3-3.24/Bullet3Common/shared/b3Int2.h
        ./bullet3-3.24/Bullet3Common/shared/b3Int4.h
        ./bullet3-3.24/Bullet3Common/shared/b3Mat3x3.h
        ./bullet3-3.24/Bullet3Common/shared/b3PlatformDefinitions.h
        ./bullet3-3.24/Bullet3Common/shared/b3Quat.h
        ./bullet3-3.24/Bullet3Dynamics/b3CpuRigidBodyPipeline.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3ContactSolverInfo.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3JacobianEntry.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3SolverBody.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3SolverConstraint.h
        ./bullet3-3.24/Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.h
        ./bullet3-3.24/Bullet3Dynamics/shared/b3ContactConstraint4.h
        ./bullet3-3.24/Bullet3Dynamics/shared/b3ConvertConstraint4.h
        ./bullet3-3.24/Bullet3Dynamics/shared/b3Inertia.h
        ./bullet3-3.24/Bullet3Dynamics/shared/b3IntegrateTransforms.h
        ./bullet3-3.24/Bullet3Geometry/b3AabbUtil.h
        ./bullet3-3.24/Bullet3Geometry/b3ConvexHullComputer.h
        ./bullet3-3.24/Bullet3Geometry/b3GeometryUtil.h
        ./bullet3-3.24/Bullet3Geometry/b3GrahamScan2dConvexHull.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuGridBroadphase.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvh.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/b3SapAabb.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphaseKernels.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvhKernels.h
        ./bullet3-3.24/Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h
        ./bullet3-3.24/Bullet3OpenCL/Initialize/b3OpenCLInclude.h
        ./bullet3-3.24/Bullet3OpenCL/Initialize/b3OpenCLUtils.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3BvhInfo.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3ConvexHullContact.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3ConvexPolyhedronCL.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3GjkEpa.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3OptimizedBvh.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3QuantizedBvh.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3StridingMeshInterface.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3SupportMappings.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3TriangleCallback.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3TriangleIndexVertexArray.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3VectorFloat4.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/b3VoronoiSimplexSolver.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h
        ./bullet3-3.24/Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3BoundSearchCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3BufferInfoCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3FillCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3PrefixScanCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3PrefixScanFloat4CL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernelsCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/CopyKernelsCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernelsCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsCL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsFloat4CL.h
        ./bullet3-3.24/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32KernelsCL.h
        ./bullet3-3.24/Bullet3OpenCL/Raycast/b3GpuRaycast.h
        ./bullet3-3.24/Bullet3OpenCL/Raycast/kernels/rayCastKernels.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuConstraint4.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuGenericConstraint.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuJacobiContactSolver.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuNarrowPhaseInternalData.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuPgsConstraintSolver.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuPgsContactSolver.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipelineInternalData.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuSolverBody.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3GpuSolverConstraint.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/b3Solver.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/batchingKernels.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/integrateKernel.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/jointSolver.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/solveContact.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/solveFriction.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/solverSetup.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/solverSetup2.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/solverUtils.h
        ./bullet3-3.24/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/autogenerated/bullet2.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3BulletFile.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Chunk.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Common.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Defines.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3DNA.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3File.h
        ./bullet3-3.24/Bullet3Serialize/Bullet2FileLoader/b3Serializer.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btAxisSweep3.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btAxisSweep3Internal.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvt.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btDispatcher.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
        ./bullet3-3.24/BulletCollision/BroadphaseCollision/btSimpleBroadphase.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btBoxBoxDetector.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionConfiguration.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionCreateFunc.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionDispatcher.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionObject.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionWorld.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionWorldImporter.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btGhostObject.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btHashedSimplePairCache.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btManifoldResult.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSimulationIslandManager.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/btUnionFind.h
        ./bullet3-3.24/BulletCollision/CollisionDispatch/SphereTriangleDetector.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBox2dShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBoxShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCapsuleShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCollisionMargin.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCollisionShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCompoundShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConcaveShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConeShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvex2dShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexHullShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexPointCloudShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexPolyhedron.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btCylinderShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btEmptyShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMaterial.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMiniSDF.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMinkowskiSumShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btMultiSphereShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btOptimizedBvh.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btSdfCollisionShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btShapeHull.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btStaticPlaneShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btStridingMeshInterface.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTetrahedronShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleBuffer.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleCallback.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleInfoMap.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMeshShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btTriangleShape.h
        ./bullet3-3.24/BulletCollision/CollisionShapes/btUniformScalingShape.h
        ./bullet3-3.24/BulletCollision/Gimpact/btBoxCollision.h
        ./bullet3-3.24/BulletCollision/Gimpact/btClipPolygon.h
        ./bullet3-3.24/BulletCollision/Gimpact/btCompoundFromGimpact.h
        ./bullet3-3.24/BulletCollision/Gimpact/btContactProcessing.h
        ./bullet3-3.24/BulletCollision/Gimpact/btContactProcessingStructs.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGenericPoolAllocator.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGeometryOperations.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactBvh.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactBvhStructs.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactMassUtil.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactQuantizedBvh.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactQuantizedBvhStructs.h
        ./bullet3-3.24/BulletCollision/Gimpact/btGImpactShape.h
        ./bullet3-3.24/BulletCollision/Gimpact/btQuantization.h
        ./bullet3-3.24/BulletCollision/Gimpact/btTriangleShapeEx.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_array.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_basic_geometry_operations.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_bitset.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_box_collision.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_box_set.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_clip_polygon.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_contact.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_geometry.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_geom_types.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_hash_table.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_linear_math.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_math.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_memory.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_pair.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_radixsort.h
        ./bullet3-3.24/BulletCollision/Gimpact/gim_tri_collision.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btConvexCast.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkCollisionDescription.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkEpa3.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btMprPenetration.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btPointCollector.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h
        ./bullet3-3.24/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
        ./bullet3-3.24/BulletDynamics/Character/btCharacterControllerInterface.h
        ./bullet3-3.24/BulletDynamics/Character/btKinematicCharacterController.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btBatchedConstraints.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btConstraintSolver.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btContactSolverInfo.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btFixedConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGearConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btHinge2Constraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btHingeConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btJacobianEntry.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSliderConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSolverBody.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btSolverConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btTypedConstraint.h
        ./bullet3-3.24/BulletDynamics/ConstraintSolver/btUniversalConstraint.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btActionInterface.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btSimpleDynamicsWorld.h
        ./bullet3-3.24/BulletDynamics/Dynamics/btSimulationIslandManagerMt.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBody.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyGearConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyInplaceSolverIslandCallback.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyJointFeedback.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyJointMotor.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyLink.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodyPoint2Point.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySliderConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySolverConstraint.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.h
        ./bullet3-3.24/BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btDantzigLCP.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btDantzigSolver.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btLemkeSolver.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btMLCPSolver.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btPATHSolver.h
        ./bullet3-3.24/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h
        ./bullet3-3.24/BulletDynamics/Vehicle/btRaycastVehicle.h
        ./bullet3-3.24/BulletDynamics/Vehicle/btVehicleRaycaster.h
        ./bullet3-3.24/BulletDynamics/Vehicle/btWheelInfo.h
        ./bullet3-3.24/BulletSoftBody/btCGProjection.h
        ./bullet3-3.24/BulletSoftBody/btConjugateGradient.h
        ./bullet3-3.24/BulletSoftBody/btConjugateResidual.h
        ./bullet3-3.24/BulletSoftBody/btDefaultSoftBodySolver.h
        ./bullet3-3.24/BulletSoftBody/btDeformableBackwardEulerObjective.h
        ./bullet3-3.24/BulletSoftBody/btDeformableBodySolver.h
        ./bullet3-3.24/BulletSoftBody/btDeformableContactConstraint.h
        ./bullet3-3.24/BulletSoftBody/btDeformableContactProjection.h
        ./bullet3-3.24/BulletSoftBody/btDeformableCorotatedForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableGravityForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableLagrangianForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableLinearElasticityForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableMassSpringForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableMousePickingForce.h
        ./bullet3-3.24/BulletSoftBody/btDeformableMultiBodyConstraintSolver.h
        ./bullet3-3.24/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h
        ./bullet3-3.24/BulletSoftBody/btDeformableNeoHookeanForce.h
        ./bullet3-3.24/BulletSoftBody/btKrylovSolver.h
        ./bullet3-3.24/BulletSoftBody/btPreconditioner.h
        ./bullet3-3.24/BulletSoftBody/btSoftBody.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodyData.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodyHelpers.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodyInternals.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodySolvers.h
        ./bullet3-3.24/BulletSoftBody/btSoftBodySolverVertexBuffer.h
        ./bullet3-3.24/BulletSoftBody/btSoftMultiBodyDynamicsWorld.h
        ./bullet3-3.24/BulletSoftBody/btSoftRigidCollisionAlgorithm.h
        ./bullet3-3.24/BulletSoftBody/btSoftRigidDynamicsWorld.h
        ./bullet3-3.24/BulletSoftBody/btSoftSoftCollisionAlgorithm.h
        ./bullet3-3.24/BulletSoftBody/btSparseSDF.h
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.h
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.h
        ./bullet3-3.24/BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableContactConstraint.h
        ./bullet3-3.24/BulletSoftBody/DeformableBodyInplaceSolverIslandCallback.h
        ./bullet3-3.24/BulletSoftBody/poly34.h
        ./bullet3-3.24/clew/clew.h
        ./bullet3-3.24/LinearMath/btAabbUtil2.h
        ./bullet3-3.24/LinearMath/btAlignedAllocator.h
        ./bullet3-3.24/LinearMath/btAlignedObjectArray.h
        ./bullet3-3.24/LinearMath/btConvexHull.h
        ./bullet3-3.24/LinearMath/btConvexHullComputer.h
        ./bullet3-3.24/LinearMath/btCpuFeatureUtility.h
        ./bullet3-3.24/LinearMath/btDefaultMotionState.h
        ./bullet3-3.24/LinearMath/btGeometryUtil.h
        ./bullet3-3.24/LinearMath/btGrahamScan2dConvexHull.h
        ./bullet3-3.24/LinearMath/btHashMap.h
        ./bullet3-3.24/LinearMath/btIDebugDraw.h
        ./bullet3-3.24/LinearMath/btImplicitQRSVD.h
        ./bullet3-3.24/LinearMath/btList.h
        ./bullet3-3.24/LinearMath/btMatrix3x3.h
        ./bullet3-3.24/LinearMath/btMatrixX.h
        ./bullet3-3.24/LinearMath/btMinMax.h
        ./bullet3-3.24/LinearMath/btModifiedGramSchmidt.h
        ./bullet3-3.24/LinearMath/btMotionState.h
        ./bullet3-3.24/LinearMath/btPolarDecomposition.h
        ./bullet3-3.24/LinearMath/btPoolAllocator.h
        ./bullet3-3.24/LinearMath/btQuadWord.h
        ./bullet3-3.24/LinearMath/btQuaternion.h
        ./bullet3-3.24/LinearMath/btQuickprof.h
        ./bullet3-3.24/LinearMath/btRandom.h
        ./bullet3-3.24/LinearMath/btReducedVector.h
        ./bullet3-3.24/LinearMath/btScalar.h
        ./bullet3-3.24/LinearMath/btSerializer.h
        ./bullet3-3.24/LinearMath/btSpatialAlgebra.h
        ./bullet3-3.24/LinearMath/btStackAlloc.h
        ./bullet3-3.24/LinearMath/btThreads.h
        ./bullet3-3.24/LinearMath/btTransform.h
        ./bullet3-3.24/LinearMath/btTransformUtil.h
        ./bullet3-3.24/LinearMath/btVector3.h
        ./bullet3-3.24/LinearMath/TaskScheduler/btThreadSupportInterface.h
        ./bullet3-3.24/btApi.h
        ./capi/capi.h)

add_library(bullet3 ${BULLET3_SOURCE_FILES} ${BULLET3_HEADER_FILES})

# install(
#   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/box2d"
#   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )