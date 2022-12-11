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

include_directories(.)

set(BULLET3_SOURCE_FILES
    # ./btBulletCollisionAll.cpp
    # ./btBulletDynamicsAll.cpp
    # ./btLinearMathAll.cpp
    ./Bullet3Collision/BroadPhaseCollision/b3DynamicBvh.cpp
    ./Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.cpp
    ./Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.cpp
    ./Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.cpp
    ./Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.cpp
    ./Bullet3Common/b3AlignedAllocator.cpp
    ./Bullet3Common/b3Logging.cpp
    ./Bullet3Common/b3Vector3.cpp
    ./Bullet3Dynamics/b3CpuRigidBodyPipeline.cpp
    ./Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.cpp
    ./Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.cpp
    ./Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.cpp
    ./Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.cpp
    ./Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.cpp
    ./Bullet3Geometry/b3ConvexHullComputer.cpp
    ./Bullet3Geometry/b3GeometryUtil.cpp
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuGridBroadphase.cpp
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvh.cpp
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.cpp
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.cpp
    # ./Bullet3OpenCL/Initialize/b3OpenCLUtils.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3ConvexHullContact.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3GjkEpa.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3OptimizedBvh.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3QuantizedBvh.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3StridingMeshInterface.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3TriangleCallback.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3TriangleIndexVertexArray.cpp
    # ./Bullet3OpenCL/NarrowphaseCollision/b3VoronoiSimplexSolver.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3BoundSearchCL.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3FillCL.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3PrefixScanCL.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3PrefixScanFloat4CL.cpp
    # ./Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.cpp
    # ./Bullet3OpenCL/Raycast/b3GpuRaycast.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuGenericConstraint.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuJacobiContactSolver.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuPgsConstraintSolver.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuPgsContactSolver.cpp
    # ./Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.cpp
    # ./Bullet3OpenCL/RigidBody/b3Solver.cpp
    ./Bullet3Serialize/Bullet2FileLoader/b3BulletFile.cpp
    ./Bullet3Serialize/Bullet2FileLoader/b3Chunk.cpp
    ./Bullet3Serialize/Bullet2FileLoader/b3DNA.cpp
    ./Bullet3Serialize/Bullet2FileLoader/b3File.cpp
    ./Bullet3Serialize/Bullet2FileLoader/b3Serializer.cpp
    ./BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
    ./BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp
    ./BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
    ./BulletCollision/BroadphaseCollision/btDbvt.cpp
    ./BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
    ./BulletCollision/BroadphaseCollision/btDispatcher.cpp
    ./BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
    ./BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
    ./BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp
    ./BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
    ./BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
    ./BulletCollision/CollisionDispatch/btCollisionDispatcherMt.cpp
    ./BulletCollision/CollisionDispatch/btCollisionObject.cpp
    ./BulletCollision/CollisionDispatch/btCollisionWorld.cpp
    ./BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp
    ./BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
    ./BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btGhostObject.cpp
    ./BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp
    ./BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp
    ./BulletCollision/CollisionDispatch/btManifoldResult.cpp
    ./BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp
    ./BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
    ./BulletCollision/CollisionDispatch/btUnionFind.cpp
    ./BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
    ./BulletCollision/CollisionShapes/btBox2dShape.cpp
    ./BulletCollision/CollisionShapes/btBoxShape.cpp
    ./BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
    ./BulletCollision/CollisionShapes/btCapsuleShape.cpp
    ./BulletCollision/CollisionShapes/btCollisionShape.cpp
    ./BulletCollision/CollisionShapes/btCompoundShape.cpp
    ./BulletCollision/CollisionShapes/btConcaveShape.cpp
    ./BulletCollision/CollisionShapes/btConeShape.cpp
    ./BulletCollision/CollisionShapes/btConvex2dShape.cpp
    ./BulletCollision/CollisionShapes/btConvexHullShape.cpp
    ./BulletCollision/CollisionShapes/btConvexInternalShape.cpp
    ./BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp
    ./BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
    ./BulletCollision/CollisionShapes/btConvexShape.cpp
    ./BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp
    ./BulletCollision/CollisionShapes/btCylinderShape.cpp
    ./BulletCollision/CollisionShapes/btEmptyShape.cpp
    ./BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp
    ./BulletCollision/CollisionShapes/btMiniSDF.cpp
    ./BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp
    ./BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp
    ./BulletCollision/CollisionShapes/btMultiSphereShape.cpp
    ./BulletCollision/CollisionShapes/btOptimizedBvh.cpp
    ./BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
    ./BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp
    ./BulletCollision/CollisionShapes/btSdfCollisionShape.cpp
    ./BulletCollision/CollisionShapes/btShapeHull.cpp
    ./BulletCollision/CollisionShapes/btSphereShape.cpp
    ./BulletCollision/CollisionShapes/btStaticPlaneShape.cpp
    ./BulletCollision/CollisionShapes/btStridingMeshInterface.cpp
    ./BulletCollision/CollisionShapes/btTetrahedronShape.cpp
    ./BulletCollision/CollisionShapes/btTriangleBuffer.cpp
    ./BulletCollision/CollisionShapes/btTriangleCallback.cpp
    ./BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp
    ./BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp
    ./BulletCollision/CollisionShapes/btTriangleMesh.cpp
    ./BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
    ./BulletCollision/CollisionShapes/btUniformScalingShape.cpp
    ./BulletCollision/Gimpact/btContactProcessing.cpp
    ./BulletCollision/Gimpact/btGenericPoolAllocator.cpp
    ./BulletCollision/Gimpact/btGImpactBvh.cpp
    ./BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp
    ./BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp
    ./BulletCollision/Gimpact/btGImpactShape.cpp
    ./BulletCollision/Gimpact/btTriangleShapeEx.cpp
    ./BulletCollision/Gimpact/gim_box_set.cpp
    ./BulletCollision/Gimpact/gim_contact.cpp
    ./BulletCollision/Gimpact/gim_memory.cpp
    ./BulletCollision/Gimpact/gim_tri_collision.cpp
    ./BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
    ./BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
    ./BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
    ./BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
    ./BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
    ./BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
    ./BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp
    ./BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
    ./BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
    ./BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
    ./BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp
    ./BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
    ./BulletDynamics/Character/btKinematicCharacterController.cpp
    ./BulletDynamics/ConstraintSolver/btBatchedConstraints.cpp
    ./BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btContactConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btFixedConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btGearConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp
    ./BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp
    ./BulletDynamics/ConstraintSolver/btHingeConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp
    ./BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
    ./BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.cpp
    ./BulletDynamics/ConstraintSolver/btSliderConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btTypedConstraint.cpp
    ./BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp
    ./BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp
    ./BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp
    ./BulletDynamics/Dynamics/btRigidBody.cpp
    ./BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp
    ./BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp
    ./BulletDynamics/Featherstone/btMultiBody.cpp
    ./BulletDynamics/Featherstone/btMultiBodyConstraint.cpp
    ./BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp
    ./BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp
    ./BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp
    ./BulletDynamics/Featherstone/btMultiBodyGearConstraint.cpp
    ./BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp
    ./BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp
    ./BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.cpp
    ./BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp
    ./BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp
    ./BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.cpp
    ./BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.cpp
    ./BulletDynamics/MLCPSolvers/btDantzigLCP.cpp
    ./BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp
    ./BulletDynamics/MLCPSolvers/btMLCPSolver.cpp
    ./BulletDynamics/Vehicle/btRaycastVehicle.cpp
    ./BulletDynamics/Vehicle/btWheelInfo.cpp
    ./BulletInverseDynamics/details/MultiBodyTreeImpl.cpp
    ./BulletInverseDynamics/details/MultiBodyTreeInitCache.cpp
    ./BulletInverseDynamics/IDMath.cpp
    ./BulletInverseDynamics/MultiBodyTree.cpp
    ./BulletSoftBody/btDefaultSoftBodySolver.cpp
    ./BulletSoftBody/btDeformableBackwardEulerObjective.cpp
    ./BulletSoftBody/btDeformableBodySolver.cpp
    ./BulletSoftBody/btDeformableContactConstraint.cpp
    ./BulletSoftBody/btDeformableContactProjection.cpp
    ./BulletSoftBody/btDeformableMultiBodyConstraintSolver.cpp
    ./BulletSoftBody/btDeformableMultiBodyDynamicsWorld.cpp
    ./BulletSoftBody/btSoftBody.cpp
    ./BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp
    ./BulletSoftBody/btSoftBodyHelpers.cpp
    ./BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp
    ./BulletSoftBody/btSoftMultiBodyDynamicsWorld.cpp
    ./BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp
    ./BulletSoftBody/btSoftRigidDynamicsWorld.cpp
    ./BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.cpp
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.cpp
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.cpp
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableContactConstraint.cpp
    ./BulletSoftBody/poly34.cpp
    ./cmake-build-debug/CMakeFiles/3.17.5/CompilerIdCXX/CMakeCXXCompilerId.cpp
    ./LinearMath/btAlignedAllocator.cpp
    ./LinearMath/btConvexHull.cpp
    ./LinearMath/btConvexHullComputer.cpp
    ./LinearMath/btGeometryUtil.cpp
    ./LinearMath/btPolarDecomposition.cpp
    ./LinearMath/btQuickprof.cpp
    ./LinearMath/btReducedVector.cpp
    ./LinearMath/btSerializer.cpp
    ./LinearMath/btSerializer64.cpp
    ./LinearMath/btThreads.cpp
    ./LinearMath/btVector3.cpp
    ./LinearMath/TaskScheduler/btTaskScheduler.cpp
    ./LinearMath/TaskScheduler/btThreadSupportPosix.cpp
    ./LinearMath/TaskScheduler/btThreadSupportWin32.cpp
    ./clew/clew.c)

set(BULLET3_HEADER_FILES
    # ./btBulletCollisionCommon.h
    # ./btBulletDynamicsCommon.h
    ./Bullet3Collision/BroadPhaseCollision/b3BroadphaseCallback.h
    ./Bullet3Collision/BroadPhaseCollision/b3DynamicBvh.h
    ./Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h
    ./Bullet3Collision/BroadPhaseCollision/b3OverlappingPair.h
    ./Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.h
    ./Bullet3Collision/BroadPhaseCollision/shared/b3Aabb.h
    ./Bullet3Collision/NarrowPhaseCollision/b3Config.h
    ./Bullet3Collision/NarrowPhaseCollision/b3Contact4.h
    ./Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h
    ./Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h
    ./Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h
    ./Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3BvhSubtreeInfoData.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3BvhTraversal.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3ClipFaces.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3Contact4Data.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3ContactConvexConvexSAT.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3ContactSphereSphere.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3ConvexPolyhedronData.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3FindConcaveSatAxis.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3FindSeparatingAxis.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3MprPenetration.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3NewContactReduction.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3QuantizedBvhNodeData.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3ReduceContacts.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h
    ./Bullet3Collision/NarrowPhaseCollision/shared/b3UpdateAabbs.h
    ./Bullet3Common/b3AlignedAllocator.h
    ./Bullet3Common/b3AlignedObjectArray.h
    ./Bullet3Common/b3CommandLineArgs.h
    ./Bullet3Common/b3FileUtils.h
    ./Bullet3Common/b3HashMap.h
    ./Bullet3Common/b3Logging.h
    ./Bullet3Common/b3Matrix3x3.h
    ./Bullet3Common/b3MinMax.h
    ./Bullet3Common/b3PoolAllocator.h
    ./Bullet3Common/b3QuadWord.h
    ./Bullet3Common/b3Quaternion.h
    ./Bullet3Common/b3Random.h
    ./Bullet3Common/b3ResizablePool.h
    ./Bullet3Common/b3Scalar.h
    ./Bullet3Common/b3StackAlloc.h
    ./Bullet3Common/b3Transform.h
    ./Bullet3Common/b3TransformUtil.h
    ./Bullet3Common/b3Vector3.h
    ./Bullet3Common/shared/b3Float4.h
    ./Bullet3Common/shared/b3Int2.h
    ./Bullet3Common/shared/b3Int4.h
    ./Bullet3Common/shared/b3Mat3x3.h
    ./Bullet3Common/shared/b3PlatformDefinitions.h
    ./Bullet3Common/shared/b3Quat.h
    ./Bullet3Dynamics/b3CpuRigidBodyPipeline.h
    ./Bullet3Dynamics/ConstraintSolver/b3ContactSolverInfo.h
    ./Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.h
    ./Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.h
    ./Bullet3Dynamics/ConstraintSolver/b3JacobianEntry.h
    ./Bullet3Dynamics/ConstraintSolver/b3PgsJacobiSolver.h
    ./Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h
    ./Bullet3Dynamics/ConstraintSolver/b3SolverBody.h
    ./Bullet3Dynamics/ConstraintSolver/b3SolverConstraint.h
    ./Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.h
    ./Bullet3Dynamics/shared/b3ContactConstraint4.h
    ./Bullet3Dynamics/shared/b3ConvertConstraint4.h
    ./Bullet3Dynamics/shared/b3Inertia.h
    ./Bullet3Dynamics/shared/b3IntegrateTransforms.h
    ./Bullet3Geometry/b3AabbUtil.h
    ./Bullet3Geometry/b3ConvexHullComputer.h
    ./Bullet3Geometry/b3GeometryUtil.h
    ./Bullet3Geometry/b3GrahamScan2dConvexHull.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuGridBroadphase.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvh.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h
    # ./Bullet3OpenCL/BroadphaseCollision/b3SapAabb.h
    # ./Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphaseKernels.h
    # ./Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvhKernels.h
    # ./Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h
    # ./Bullet3OpenCL/Initialize/b3OpenCLInclude.h
    # ./Bullet3OpenCL/Initialize/b3OpenCLUtils.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3BvhInfo.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3ConvexHullContact.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3ConvexPolyhedronCL.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3GjkEpa.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3OptimizedBvh.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3QuantizedBvh.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3StridingMeshInterface.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3SupportMappings.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3TriangleCallback.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3TriangleIndexVertexArray.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3VectorFloat4.h
    # ./Bullet3OpenCL/NarrowphaseCollision/b3VoronoiSimplexSolver.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h
    # ./Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3BoundSearchCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3BufferInfoCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3FillCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3PrefixScanCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3PrefixScanFloat4CL.h
    # ./Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernelsCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/CopyKernelsCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/FillKernelsCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsCL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsFloat4CL.h
    # ./Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32KernelsCL.h
    # ./Bullet3OpenCL/Raycast/b3GpuRaycast.h
    # ./Bullet3OpenCL/Raycast/kernels/rayCastKernels.h
    # ./Bullet3OpenCL/RigidBody/b3GpuConstraint4.h
    # ./Bullet3OpenCL/RigidBody/b3GpuGenericConstraint.h
    # ./Bullet3OpenCL/RigidBody/b3GpuJacobiContactSolver.h
    # ./Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h
    # ./Bullet3OpenCL/RigidBody/b3GpuNarrowPhaseInternalData.h
    # ./Bullet3OpenCL/RigidBody/b3GpuPgsConstraintSolver.h
    # ./Bullet3OpenCL/RigidBody/b3GpuPgsContactSolver.h
    # ./Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h
    # ./Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipelineInternalData.h
    # ./Bullet3OpenCL/RigidBody/b3GpuSolverBody.h
    # ./Bullet3OpenCL/RigidBody/b3GpuSolverConstraint.h
    # ./Bullet3OpenCL/RigidBody/b3Solver.h
    # ./Bullet3OpenCL/RigidBody/kernels/batchingKernels.h
    # ./Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.h
    # ./Bullet3OpenCL/RigidBody/kernels/integrateKernel.h
    # ./Bullet3OpenCL/RigidBody/kernels/jointSolver.h
    # ./Bullet3OpenCL/RigidBody/kernels/solveContact.h
    # ./Bullet3OpenCL/RigidBody/kernels/solveFriction.h
    # ./Bullet3OpenCL/RigidBody/kernels/solverSetup.h
    # ./Bullet3OpenCL/RigidBody/kernels/solverSetup2.h
    # ./Bullet3OpenCL/RigidBody/kernels/solverUtils.h
    # ./Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h
    ./Bullet3Serialize/Bullet2FileLoader/autogenerated/bullet2.h
    ./Bullet3Serialize/Bullet2FileLoader/b3BulletFile.h
    ./Bullet3Serialize/Bullet2FileLoader/b3Chunk.h
    ./Bullet3Serialize/Bullet2FileLoader/b3Common.h
    ./Bullet3Serialize/Bullet2FileLoader/b3Defines.h
    ./Bullet3Serialize/Bullet2FileLoader/b3DNA.h
    ./Bullet3Serialize/Bullet2FileLoader/b3File.h
    ./Bullet3Serialize/Bullet2FileLoader/b3Serializer.h
    ./BulletCollision/BroadphaseCollision/btAxisSweep3.h
    ./BulletCollision/BroadphaseCollision/btAxisSweep3Internal.h
    ./BulletCollision/BroadphaseCollision/btBroadphaseInterface.h
    ./BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
    ./BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
    ./BulletCollision/BroadphaseCollision/btDbvt.h
    ./BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
    ./BulletCollision/BroadphaseCollision/btDispatcher.h
    ./BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
    ./BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h
    ./BulletCollision/BroadphaseCollision/btQuantizedBvh.h
    ./BulletCollision/BroadphaseCollision/btSimpleBroadphase.h
    ./BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btBoxBoxDetector.h
    ./BulletCollision/CollisionDispatch/btCollisionConfiguration.h
    ./BulletCollision/CollisionDispatch/btCollisionCreateFunc.h
    ./BulletCollision/CollisionDispatch/btCollisionDispatcher.h
    ./BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h
    ./BulletCollision/CollisionDispatch/btCollisionObject.h
    ./BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h
    ./BulletCollision/CollisionDispatch/btCollisionWorld.h
    ./BulletCollision/CollisionDispatch/btCollisionWorldImporter.h
    ./BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h
    ./BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
    ./BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
    ./BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btGhostObject.h
    ./BulletCollision/CollisionDispatch/btHashedSimplePairCache.h
    ./BulletCollision/CollisionDispatch/btInternalEdgeUtility.h
    ./BulletCollision/CollisionDispatch/btManifoldResult.h
    ./BulletCollision/CollisionDispatch/btSimulationIslandManager.h
    ./BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h
    ./BulletCollision/CollisionDispatch/btUnionFind.h
    ./BulletCollision/CollisionDispatch/SphereTriangleDetector.h
    ./BulletCollision/CollisionShapes/btBox2dShape.h
    ./BulletCollision/CollisionShapes/btBoxShape.h
    ./BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
    ./BulletCollision/CollisionShapes/btCapsuleShape.h
    ./BulletCollision/CollisionShapes/btCollisionMargin.h
    ./BulletCollision/CollisionShapes/btCollisionShape.h
    ./BulletCollision/CollisionShapes/btCompoundShape.h
    ./BulletCollision/CollisionShapes/btConcaveShape.h
    ./BulletCollision/CollisionShapes/btConeShape.h
    ./BulletCollision/CollisionShapes/btConvex2dShape.h
    ./BulletCollision/CollisionShapes/btConvexHullShape.h
    ./BulletCollision/CollisionShapes/btConvexInternalShape.h
    ./BulletCollision/CollisionShapes/btConvexPointCloudShape.h
    ./BulletCollision/CollisionShapes/btConvexPolyhedron.h
    ./BulletCollision/CollisionShapes/btConvexShape.h
    ./BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h
    ./BulletCollision/CollisionShapes/btCylinderShape.h
    ./BulletCollision/CollisionShapes/btEmptyShape.h
    ./BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h
    ./BulletCollision/CollisionShapes/btMaterial.h
    ./BulletCollision/CollisionShapes/btMiniSDF.h
    ./BulletCollision/CollisionShapes/btMinkowskiSumShape.h
    ./BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h
    ./BulletCollision/CollisionShapes/btMultiSphereShape.h
    ./BulletCollision/CollisionShapes/btOptimizedBvh.h
    ./BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
    ./BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h
    ./BulletCollision/CollisionShapes/btSdfCollisionShape.h
    ./BulletCollision/CollisionShapes/btShapeHull.h
    ./BulletCollision/CollisionShapes/btSphereShape.h
    ./BulletCollision/CollisionShapes/btStaticPlaneShape.h
    ./BulletCollision/CollisionShapes/btStridingMeshInterface.h
    ./BulletCollision/CollisionShapes/btTetrahedronShape.h
    ./BulletCollision/CollisionShapes/btTriangleBuffer.h
    ./BulletCollision/CollisionShapes/btTriangleCallback.h
    ./BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h
    ./BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h
    ./BulletCollision/CollisionShapes/btTriangleInfoMap.h
    ./BulletCollision/CollisionShapes/btTriangleMesh.h
    ./BulletCollision/CollisionShapes/btTriangleMeshShape.h
    ./BulletCollision/CollisionShapes/btTriangleShape.h
    ./BulletCollision/CollisionShapes/btUniformScalingShape.h
    ./BulletCollision/Gimpact/btBoxCollision.h
    ./BulletCollision/Gimpact/btClipPolygon.h
    ./BulletCollision/Gimpact/btCompoundFromGimpact.h
    ./BulletCollision/Gimpact/btContactProcessing.h
    ./BulletCollision/Gimpact/btContactProcessingStructs.h
    ./BulletCollision/Gimpact/btGenericPoolAllocator.h
    ./BulletCollision/Gimpact/btGeometryOperations.h
    ./BulletCollision/Gimpact/btGImpactBvh.h
    ./BulletCollision/Gimpact/btGImpactBvhStructs.h
    ./BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h
    ./BulletCollision/Gimpact/btGImpactMassUtil.h
    ./BulletCollision/Gimpact/btGImpactQuantizedBvh.h
    ./BulletCollision/Gimpact/btGImpactQuantizedBvhStructs.h
    ./BulletCollision/Gimpact/btGImpactShape.h
    ./BulletCollision/Gimpact/btQuantization.h
    ./BulletCollision/Gimpact/btTriangleShapeEx.h
    ./BulletCollision/Gimpact/gim_array.h
    ./BulletCollision/Gimpact/gim_basic_geometry_operations.h
    ./BulletCollision/Gimpact/gim_bitset.h
    ./BulletCollision/Gimpact/gim_box_collision.h
    ./BulletCollision/Gimpact/gim_box_set.h
    ./BulletCollision/Gimpact/gim_clip_polygon.h
    ./BulletCollision/Gimpact/gim_contact.h
    ./BulletCollision/Gimpact/gim_geometry.h
    ./BulletCollision/Gimpact/gim_geom_types.h
    ./BulletCollision/Gimpact/gim_hash_table.h
    ./BulletCollision/Gimpact/gim_linear_math.h
    ./BulletCollision/Gimpact/gim_math.h
    ./BulletCollision/Gimpact/gim_memory.h
    ./BulletCollision/Gimpact/gim_pair.h
    ./BulletCollision/Gimpact/gim_radixsort.h
    ./BulletCollision/Gimpact/gim_tri_collision.h
    ./BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h
    ./BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h
    ./BulletCollision/NarrowPhaseCollision/btConvexCast.h
    ./BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h
    ./BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h
    ./BulletCollision/NarrowPhaseCollision/btGjkCollisionDescription.h
    ./BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h
    ./BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
    ./BulletCollision/NarrowPhaseCollision/btGjkEpa3.h
    ./BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
    ./BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
    ./BulletCollision/NarrowPhaseCollision/btManifoldPoint.h
    ./BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
    ./BulletCollision/NarrowPhaseCollision/btMprPenetration.h
    ./BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
    ./BulletCollision/NarrowPhaseCollision/btPointCollector.h
    ./BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h
    ./BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
    ./BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h
    ./BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h
    ./BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
    ./BulletDynamics/Character/btCharacterControllerInterface.h
    ./BulletDynamics/Character/btKinematicCharacterController.h
    ./BulletDynamics/ConstraintSolver/btBatchedConstraints.h
    ./BulletDynamics/ConstraintSolver/btConeTwistConstraint.h
    ./BulletDynamics/ConstraintSolver/btConstraintSolver.h
    ./BulletDynamics/ConstraintSolver/btContactConstraint.h
    ./BulletDynamics/ConstraintSolver/btContactSolverInfo.h
    ./BulletDynamics/ConstraintSolver/btFixedConstraint.h
    ./BulletDynamics/ConstraintSolver/btGearConstraint.h
    ./BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h
    ./BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h
    ./BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h
    ./BulletDynamics/ConstraintSolver/btHinge2Constraint.h
    ./BulletDynamics/ConstraintSolver/btHingeConstraint.h
    ./BulletDynamics/ConstraintSolver/btJacobianEntry.h
    ./BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h
    ./BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h
    ./BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h
    ./BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h
    ./BulletDynamics/ConstraintSolver/btSliderConstraint.h
    ./BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h
    ./BulletDynamics/ConstraintSolver/btSolverBody.h
    ./BulletDynamics/ConstraintSolver/btSolverConstraint.h
    ./BulletDynamics/ConstraintSolver/btTypedConstraint.h
    ./BulletDynamics/ConstraintSolver/btUniversalConstraint.h
    ./BulletDynamics/Dynamics/btActionInterface.h
    ./BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h
    ./BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h
    ./BulletDynamics/Dynamics/btDynamicsWorld.h
    ./BulletDynamics/Dynamics/btRigidBody.h
    ./BulletDynamics/Dynamics/btSimpleDynamicsWorld.h
    ./BulletDynamics/Dynamics/btSimulationIslandManagerMt.h
    ./BulletDynamics/Featherstone/btMultiBody.h
    ./BulletDynamics/Featherstone/btMultiBodyConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h
    ./BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h
    ./BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodyGearConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodyInplaceSolverIslandCallback.h
    ./BulletDynamics/Featherstone/btMultiBodyJointFeedback.h
    ./BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodyJointMotor.h
    ./BulletDynamics/Featherstone/btMultiBodyLink.h
    ./BulletDynamics/Featherstone/btMultiBodyLinkCollider.h
    ./BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h
    ./BulletDynamics/Featherstone/btMultiBodyPoint2Point.h
    ./BulletDynamics/Featherstone/btMultiBodySliderConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodySolverConstraint.h
    ./BulletDynamics/Featherstone/btMultiBodySphericalJointLimit.h
    ./BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h
    ./BulletDynamics/MLCPSolvers/btDantzigLCP.h
    ./BulletDynamics/MLCPSolvers/btDantzigSolver.h
    ./BulletDynamics/MLCPSolvers/btLemkeAlgorithm.h
    ./BulletDynamics/MLCPSolvers/btLemkeSolver.h
    ./BulletDynamics/MLCPSolvers/btMLCPSolver.h
    ./BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h
    ./BulletDynamics/MLCPSolvers/btPATHSolver.h
    ./BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h
    ./BulletDynamics/Vehicle/btRaycastVehicle.h
    ./BulletDynamics/Vehicle/btVehicleRaycaster.h
    ./BulletDynamics/Vehicle/btWheelInfo.h
    ./BulletSoftBody/btCGProjection.h
    ./BulletSoftBody/btConjugateGradient.h
    ./BulletSoftBody/btConjugateResidual.h
    ./BulletSoftBody/btDefaultSoftBodySolver.h
    ./BulletSoftBody/btDeformableBackwardEulerObjective.h
    ./BulletSoftBody/btDeformableBodySolver.h
    ./BulletSoftBody/btDeformableContactConstraint.h
    ./BulletSoftBody/btDeformableContactProjection.h
    ./BulletSoftBody/btDeformableCorotatedForce.h
    ./BulletSoftBody/btDeformableGravityForce.h
    ./BulletSoftBody/btDeformableLagrangianForce.h
    ./BulletSoftBody/btDeformableLinearElasticityForce.h
    ./BulletSoftBody/btDeformableMassSpringForce.h
    ./BulletSoftBody/btDeformableMousePickingForce.h
    ./BulletSoftBody/btDeformableMultiBodyConstraintSolver.h
    ./BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h
    ./BulletSoftBody/btDeformableNeoHookeanForce.h
    ./BulletSoftBody/btKrylovSolver.h
    ./BulletSoftBody/btPreconditioner.h
    ./BulletSoftBody/btSoftBody.h
    ./BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.h
    ./BulletSoftBody/btSoftBodyData.h
    ./BulletSoftBody/btSoftBodyHelpers.h
    ./BulletSoftBody/btSoftBodyInternals.h
    ./BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h
    ./BulletSoftBody/btSoftBodySolvers.h
    ./BulletSoftBody/btSoftBodySolverVertexBuffer.h
    ./BulletSoftBody/btSoftMultiBodyDynamicsWorld.h
    ./BulletSoftBody/btSoftRigidCollisionAlgorithm.h
    ./BulletSoftBody/btSoftRigidDynamicsWorld.h
    ./BulletSoftBody/btSoftSoftCollisionAlgorithm.h
    ./BulletSoftBody/btSparseSDF.h
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.h
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.h
    ./BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableContactConstraint.h
    ./BulletSoftBody/DeformableBodyInplaceSolverIslandCallback.h
    ./BulletSoftBody/poly34.h
    ./clew/clew.h
    ./LinearMath/btAabbUtil2.h
    ./LinearMath/btAlignedAllocator.h
    ./LinearMath/btAlignedObjectArray.h
    ./LinearMath/btConvexHull.h
    ./LinearMath/btConvexHullComputer.h
    ./LinearMath/btCpuFeatureUtility.h
    ./LinearMath/btDefaultMotionState.h
    ./LinearMath/btGeometryUtil.h
    ./LinearMath/btGrahamScan2dConvexHull.h
    ./LinearMath/btHashMap.h
    ./LinearMath/btIDebugDraw.h
    ./LinearMath/btImplicitQRSVD.h
    ./LinearMath/btList.h
    ./LinearMath/btMatrix3x3.h
    ./LinearMath/btMatrixX.h
    ./LinearMath/btMinMax.h
    ./LinearMath/btModifiedGramSchmidt.h
    ./LinearMath/btMotionState.h
    ./LinearMath/btPolarDecomposition.h
    ./LinearMath/btPoolAllocator.h
    ./LinearMath/btQuadWord.h
    ./LinearMath/btQuaternion.h
    ./LinearMath/btQuickprof.h
    ./LinearMath/btRandom.h
    ./LinearMath/btReducedVector.h
    ./LinearMath/btScalar.h
    ./LinearMath/btSerializer.h
    ./LinearMath/btSpatialAlgebra.h
    ./LinearMath/btStackAlloc.h
    ./LinearMath/btThreads.h
    ./LinearMath/btTransform.h
    ./LinearMath/btTransformUtil.h
    ./LinearMath/btVector3.h
    ./LinearMath/TaskScheduler/btThreadSupportInterface.h
    ./btApi.h)

add_library(bullet3 ${BULLET3_SOURCE_FILES} ${BULLET3_HEADER_FILES})

# install(
#   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/box2d"
#   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )